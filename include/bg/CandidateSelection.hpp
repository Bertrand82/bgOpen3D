#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "ColmapModel.hpp"

namespace loc {

struct CandidateSelectionOptions {
  int max_candidates = 5;
};

class ICandidateSelector {
public:
  virtual ~ICandidateSelector() = default;

  virtual std::vector<int> SelectCandidates(const ColmapReconstruction& model,
                                            const Eigen::Vector3d& init_center_world) const = 0;
};

class HardcodedCandidateSelector final : public ICandidateSelector {
public:
  explicit HardcodedCandidateSelector(CandidateSelectionOptions opt = {}) : opt_(opt) {}

  std::vector<int> SelectCandidates(const ColmapReconstruction& model,
                                    const Eigen::Vector3d&) const override {
    std::vector<int> ids;
    ids.reserve(model.images.size());
    for (const auto& kv : model.images) {
      ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());

    const std::size_t max_keep =
        opt_.max_candidates > 0 ? static_cast<std::size_t>(opt_.max_candidates) : std::size_t{0};
    if (ids.size() > max_keep) {
      ids.resize(max_keep);
    }
    return ids;
  }

private:
  CandidateSelectionOptions opt_;
};

}  // namespace loc
