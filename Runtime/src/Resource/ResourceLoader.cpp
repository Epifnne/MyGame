#include "Resource/ResourceLoader.h"

namespace Runtime {
namespace Resource {

bool ResourceLoader::Upload(const AssetMetadata&, const std::shared_ptr<Resource>& resource) const {
    return resource != nullptr;
}

} // namespace Resource
} // namespace Runtime
