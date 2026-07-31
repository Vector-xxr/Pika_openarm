// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ik/ik_solver.hpp>

#include <ik/dls_ik.hpp>

#include <algorithm>
#include <cctype>

namespace {

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

}  // namespace

std::unique_ptr<IkSolver> create_ik_solver(const std::string& name, Dynamics* dyn,
                                           const CartesianControllerConfig& cfg) {
    const std::string key = to_lower(name);
    if (key.empty() || key == "dls") {
        return std::make_unique<DlsIk>(dyn, cfg);
    }
    return nullptr;
}
