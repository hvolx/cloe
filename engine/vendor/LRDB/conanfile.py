# mypy: ignore-errors
# pylint: skip-file

import os
from pathlib import Path

from conan import ConanFile
from conans import tools
from conan.tools import cmake, env, files, scm

from pprint import pprint

required_conan_version = ">=1.52.0"

class LRDB(ConanFile):
    name = "LRDB"
    version = "latest"
    license = "BSL-1.0"
    url = "https://github.com/satoren/LRDB"
    description = "Lua Remote DeBugger"
    settings = "os", "compiler", "build_type", "arch"

    generators = "CMakeDeps", "VirtualRunEnv"
    exports_sources = [
        "LRDB-main/include/*",
        "LRDB-main/third_party/*"
    ]

    _git_url = "https://github.com/satoren/LRDB.git"
    _git_dir = "LRDB-main"
    _git_ref = "main" if version == "latest" else f"v{version}"    

    def requirements(self):
        self.requires("lua/[>=5.0.0]")

    def export_sources(self):
        #tools.unzip(<path>, destination=".", keep_permissions=False, pattern=None, strip_root=False)
        scm.Git(self).clone(url=self._git_url, target="LRDB-main")

    def package(self):
        #pprint(vars(self))
        files.copy(self, "*.*", "./LRDB-main/include", dst=self.package_folder+"/include")
        files.copy(self, "picojson.h", "./LRDB-main/third_party/picojson", dst=self.package_folder+"/include")

    # def package_info(self):
    #     self.cpp_info.set_property("cmake_find_mode", "both")
    #     self.cpp_info.set_property("cmake_file_name", "cloe-engine")
    #     self.cpp_info.set_property("cmake_target_name", "cloe::engine")
    #     self.cpp_info.set_property("pkg_config_name", "cloe-engine")
    #     if self.settings.os == "Linux":
    #         self.cpp_info.system_libs.append("pthread")
    #         self.cpp_info.system_libs.append("dl")
    #     if self.in_local_cache:
    #         bindir = os.path.join(self.package_folder, "bin")
    #         luadir = os.path.join(self.package_folder, "lib/cloe/lua")
    #     else:
    #         bindir = os.path.join(self.build_folder, "bin")
    #         luadir = os.path.join(self.source_folder, "lua")

    #     self.output.info(f"Appending PATH environment variable: {bindir}")
    #     self.runenv_info.prepend_path("PATH", bindir)
    #     self.output.info(f"Appending CLOE_LUA_PATH environment variable: {luadir}")
    #     self.runenv_info.prepend_path("CLOE_LUA_PATH", luadir)
