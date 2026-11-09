#include "naex/grid/map_file.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>

namespace naex {
namespace grid {

namespace {

constexpr char kMagic[8] = {'N', 'A', 'E', 'X', 'G', 'R', 'I', 'D'};
constexpr uint32_t kFormatVersion = 1;

// The cells are dumped as raw memory.
static_assert(std::is_trivially_copyable<Cell>::value && sizeof(Cell) == 4);
static_assert(std::is_trivially_copyable<Costs>::value &&
              sizeof(Costs) == 4 * sizeof(float));

/**
 * Cell indices in_cell_range() accepts.  Staying inside it also keeps
 * neighbor8() from wrapping around at the ends of the int16_t range.
 */
constexpr double kMinCellIndex = -32767.0;
constexpr double kMaxCellIndexInclusive = 32766.0;

bool is_addressable(double index) {
  return index >= kMinCellIndex && index <= kMaxCellIndexInclusive;
}

[[noreturn]] void throw_errno(const std::string &what) {
  throw std::runtime_error(what + ": " + std::strerror(errno));
}

template <typename T> void put(std::string &out, const T &value) {
  out.append(reinterpret_cast<const char *>(&value), sizeof(T));
}

void put_string(std::string &out, const std::string &value) {
  put(out, static_cast<uint32_t>(value.size()));
  out.append(value);
}

template <typename T>
void put_vector(std::string &out, const std::vector<T> &values) {
  out.append(reinterpret_cast<const char *>(values.data()),
             values.size() * sizeof(T));
}

/// Bounds-checked reader; a short file throws instead of reading past the end.
class Reader {
public:
  explicit Reader(const std::string &data) : data_(data) {}

  size_t remaining() const { return data_.size() - pos_; }

  void bytes(void *out, size_t n) {
    if (n > remaining()) {
      throw std::runtime_error("file is truncated");
    }
    std::memcpy(out, data_.data() + pos_, n);
    pos_ += n;
  }

  template <typename T> T value() {
    T v;
    bytes(&v, sizeof(T));
    return v;
  }

  std::string string() {
    const uint32_t n = value<uint32_t>();
    if (n > remaining()) {
      throw std::runtime_error("file is truncated");
    }
    std::string s(n, '\0');
    bytes(s.data(), n);
    return s;
  }

  /// Checks the size first, so a corrupt count cannot trigger a huge
  /// allocation.
  template <typename T> std::vector<T> vector(uint64_t n) {
    if (n > remaining() / sizeof(T)) {
      throw std::runtime_error("file is truncated");
    }
    std::vector<T> v(static_cast<size_t>(n));
    bytes(v.data(), static_cast<size_t>(n) * sizeof(T));
    return v;
  }

private:
  const std::string &data_;
  size_t pos_{0};
};

std::string serialize(const MapSnapshot &map) {
  const size_t num_cells = map.cells.cells.size();
  std::string out;
  out.reserve(1024 + num_cells * (sizeof(Cell) + sizeof(Costs)));
  out.append(kMagic, sizeof(kMagic));
  put(out, kFormatVersion);
  put(out, map.config.cell_size);
  put_string(out, map.config.earth_frame);
  put_string(out, map.map_frame);
  put(out, map.earth_from_map);
  put(out, map.config.default_costs);
  put(out, static_cast<uint32_t>(map.config.cost_fields.size()));
  for (const std::string &field : map.config.cost_fields) {
    put_string(out, field);
  }
  put(out, static_cast<uint32_t>(map.config.cloud_levels.size()));
  put_vector(out, map.config.cloud_levels);
  put(out, static_cast<uint32_t>(map.config.cloud_weights.size()));
  put_vector(out, map.config.cloud_weights);
  put(out, static_cast<uint64_t>(num_cells));
  put_vector(out, map.cells.cells);
  put_vector(out, map.cells.costs);
  return out;
}

/// Closes a file descriptor on every path out of write_and_sync().
class Fd {
public:
  explicit Fd(int fd) : fd_(fd) {}
  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;
  ~Fd() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  int get() const { return fd_; }

private:
  int fd_;
};

void write_and_sync(const std::string &path, const std::string &data) {
  const Fd fd(
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644));
  if (fd.get() < 0) {
    throw_errno("cannot open " + path);
  }
  size_t written = 0;
  while (written < data.size()) {
    const ssize_t n =
        ::write(fd.get(), data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw_errno("cannot write " + path);
    }
    written += static_cast<size_t>(n);
  }
  if (::fsync(fd.get()) != 0) {
    throw_errno("cannot fsync " + path);
  }
}

} // namespace

std::string describe_config_mismatch(const MapConfig &saved,
                                     const MapConfig &current) {
  std::ostringstream ss;
  if (saved.earth_frame != current.earth_frame) {
    ss << "earth_frame is '" << saved.earth_frame << "' in the file, '"
       << current.earth_frame << "' now";
  } else if (saved.cell_size != current.cell_size) {
    ss << "cell_size is " << saved.cell_size << " in the file, "
       << current.cell_size << " now";
  } else if (saved.cost_fields != current.cost_fields) {
    ss << "cost_fields differ";
  } else if (saved.cloud_levels != current.cloud_levels) {
    ss << "cloud_levels differ";
  } else if (saved.cloud_weights != current.cloud_weights) {
    ss << "cloud_weights differ";
  } else {
    for (size_t i = 0; i < Costs::kSize; ++i) {
      const Cost a = saved.default_costs[i];
      const Cost b = current.default_costs[i];
      if (a != b && !(std::isnan(a) && std::isnan(b))) {
        ss << "default_costs[" << i << "] is " << a << " in the file, " << b
           << " now";
        break;
      }
    }
  }
  return ss.str();
}

void write_map_file(const std::string &path, const MapSnapshot &map) {
  if (map.cells.cells.size() != map.cells.costs.size()) {
    throw std::runtime_error("cells and costs differ in size");
  }
  namespace fs = std::filesystem;
  fs::path dir = fs::path(path).parent_path();
  if (dir.empty()) {
    dir = ".";
  }
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    throw std::runtime_error("cannot create " + dir.string() + ": " +
                             ec.message());
  }

  const std::string tmp = path + ".tmp";
  try {
    write_and_sync(tmp, serialize(map));
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
      throw_errno("cannot rename " + tmp + " to " + path);
    }
  } catch (...) {
    ::unlink(tmp.c_str());
    throw;
  }

  // Without this the rename itself may not survive a power loss.  The new file
  // is complete either way, so a failure here is not worth reporting.
  const Fd dir_fd(::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (dir_fd.get() >= 0) {
    ::fsync(dir_fd.get());
  }
}

MapSnapshot read_map_file(const std::string &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw_errno("cannot open " + path);
  }
  const std::string data((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  if (file.bad()) {
    throw std::runtime_error("cannot read " + path);
  }

  Reader in(data);
  char magic[sizeof(kMagic)];
  in.bytes(magic, sizeof(magic));
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    throw std::runtime_error("not a naex map file");
  }
  const uint32_t version = in.value<uint32_t>();
  if (version != kFormatVersion) {
    throw std::runtime_error("unsupported format version " +
                             std::to_string(version));
  }

  MapSnapshot map;
  map.config.cell_size = in.value<float>();
  map.config.earth_frame = in.string();
  map.map_frame = in.string();
  map.earth_from_map = in.value<MapPose>();
  map.config.default_costs = in.value<Costs>();
  const uint32_t num_fields = in.value<uint32_t>();
  for (uint32_t i = 0; i < num_fields; ++i) {
    map.config.cost_fields.push_back(in.string());
  }
  map.config.cloud_levels = in.vector<int64_t>(in.value<uint32_t>());
  map.config.cloud_weights = in.vector<double>(in.value<uint32_t>());

  const uint64_t num_cells = in.value<uint64_t>();
  map.cells.cells = in.vector<Cell>(num_cells);
  map.cells.costs = in.vector<Costs>(num_cells);
  if (in.remaining() != 0) {
    throw std::runtime_error("file has trailing bytes");
  }
  return map;
}

MapCells copy_cells(const Grid &grid) {
  MapCells out;
  const CellId n = static_cast<CellId>(grid.size());
  out.cells.reserve(n);
  out.costs.reserve(n);
  for (CellId v = 0; v < n; ++v) {
    out.cells.push_back(grid.cell(v));
    out.costs.push_back(grid.costs(v));
  }
  return out;
}

Rigid2 relative_rigid2(const MapPose &earth_from_old,
                       const MapPose &earth_from_new) {
  const auto to_isometry = [](const MapPose &p) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
    t.linear() = Eigen::Quaterniond(p[6], p[3], p[4], p[5])
                     .normalized()
                     .toRotationMatrix();
    return t;
  };
  // Doubles throughout: the two poses are ~6.4e6 m from the earth origin and
  // only their difference is small.
  const Eigen::Isometry3d t =
      to_isometry(earth_from_new).inverse() * to_isometry(earth_from_old);
  const double yaw = std::atan2(t.linear()(1, 0), t.linear()(0, 0));
  Rigid2 out;
  out.c = std::cos(yaw);
  out.s = std::sin(yaw);
  out.tx = t.translation().x();
  out.ty = t.translation().y();
  return out;
}

MapCells resample_cells(const MapCells &old, float cell_size,
                        const Rigid2 &new_from_old) {
  const double size = cell_size;
  const Rigid2 &t = new_from_old;
  const size_t n = old.cells.size();

  std::unordered_map<Cell, size_t, CellHasher> old_index;
  old_index.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    old_index.emplace(old.cells[i], i);
  }

  MapCells out;
  out.cells.reserve(n);
  out.costs.reserve(n);
  std::unordered_set<Cell, CellHasher> tried;
  tried.reserve(2 * n);

  // Walking the bounding box of the target would be up to 4e9 lookups on a long
  // sparse route, so only the candidates around each old cell are visited.  A
  // target centre inside an old cell is at most sqrt(2)/2 cells from that old
  // centre under any rotation, hence in the 3x3 block around where it lands.
  for (size_t i = 0; i < n; ++i) {
    const double ox = (old.cells[i].x + 0.5) * size;
    const double oy = (old.cells[i].y + 0.5) * size;
    const double cx = std::floor((t.c * ox - t.s * oy + t.tx) / size);
    const double cy = std::floor((t.s * ox + t.c * oy + t.ty) / size);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const double tx = cx + dx;
        const double ty = cy + dy;
        if (!is_addressable(tx) || !is_addressable(ty)) {
          continue;
        }
        const Cell target(static_cast<int16_t>(tx), static_cast<int16_t>(ty));
        if (!tried.insert(target).second) {
          continue;
        }
        // Back to the old frame: the inverse rotation of (centre - t).
        const double qx = (tx + 0.5) * size - t.tx;
        const double qy = (ty + 0.5) * size - t.ty;
        const double sx = std::floor((t.c * qx + t.s * qy) / size);
        const double sy = std::floor((-t.s * qx + t.c * qy) / size);
        if (!is_addressable(sx) || !is_addressable(sy)) {
          continue;
        }
        const auto source = old_index.find(
            Cell(static_cast<int16_t>(sx), static_cast<int16_t>(sy)));
        if (source != old_index.end()) {
          out.cells.push_back(target);
          out.costs.push_back(old.costs[source->second]);
        }
      }
    }
  }
  return out;
}

size_t merge_cells(Grid &grid, const MapCells &cells) {
  grid.reserve(grid.size() + cells.cells.size());
  size_t added = 0;
  for (size_t i = 0; i < cells.cells.size(); ++i) {
    if (grid.has_cell(cells.cells[i])) {
      continue;
    }
    grid.cell_costs(cells.cells[i]) = cells.costs[i];
    ++added;
  }
  return added;
}

} // namespace grid
} // namespace naex
