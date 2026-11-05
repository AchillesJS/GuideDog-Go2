// SPDX-License-Identifier: Apache-2.0
// Offline exporter: use Gazebo's own mesh loader, including COLLADA transforms.
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  const auto *mesh = gz::common::MeshManager::Instance()->Load(argv[1]);
  if (!mesh || mesh->VertexCount() == 0) return 3;
  const auto low = mesh->Min();
  const auto high = mesh->Max();
  std::cout << std::setprecision(17) << "GUIDEDOG_BOUNDS {\"min\":["
    << low.X() << "," << low.Y() << "," << low.Z() << "],\"max\":["
    << high.X() << "," << high.Y() << "," << high.Z() << "],\"vertices\":"
    << mesh->VertexCount() << "}\n";
}
