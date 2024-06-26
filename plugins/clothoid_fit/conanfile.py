# mypy: ignore-errors
# pylint: skip-file

from pathlib import Path

from conan import ConanFile
from conan.tools import cmake, files, scm

required_conan_version = ">=1.52.0"

class CloeComponentClothoidFit(ConanFile):
    name = "cloe-plugin-clothoid-fit"
    url = "https://github.com/eclipse/cloe"
    description = "Cloe component plugin that generates clothoids from given lane boundary point lists."
    license = "Apache-2.0"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "VirtualRunEnv"
    no_copy_source = True
    exports_sources = [
        "src/*",
        "CMakeLists.txt",
    ]

    def set_version(self):
        version_file = Path(self.recipe_folder) / "../../VERSION"
        if version_file.exists():
            self.version = files.load(self, version_file).strip()
        else:
            git = scm.Git(self, self.recipe_folder)
            self.version = git.run("describe --dirty=-dirty")[1:]

    def requirements(self):
        self.requires(f"cloe-runtime/{self.version}@cloe/develop")
        self.requires(f"cloe-models/{self.version}@cloe/develop")
        self.requires("eigen/3.4.0")

    def build_requirements(self):
        self.test_requires("gtest/1.14.0")

    def layout(self):
        cmake.cmake_layout(self)

    def generate(self):
        tc = cmake.CMakeToolchain(self)
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.cache_variables["CLOE_PROJECT_VERSION"] = self.version
        tc.generate()

    def build(self):
        cm = cmake.CMake(self)
        if self.should_configure:
            cm.configure()
        if self.should_build:
            cm.build()
        if self.should_test:
            cm.test()

    def package(self):
        cm = cmake.CMake(self)
        if self.should_install:
            cm.install()

    def package_id(self):
        self.info.requires["boost"].full_package_mode()

    def package_info(self):
        self.cpp_info.set_property("cmake_find_mode", "both")
        self.cpp_info.set_property("cmake_file_name", self.name)
        self.cpp_info.set_property("pkg_config_name", self.name)
