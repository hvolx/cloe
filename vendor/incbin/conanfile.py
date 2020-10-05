from conans import ConanFile, CMake, tools
import shutil
import os.path


class IncbinConan(ConanFile):
    name = "incbin"
    # Note that the below version 1.X.Y is a fictional version, where X is the
    # number of commits in the repository and Y is the latest iteration of our
    # changes. This is necessary because incbin doesn't have a version.
    version = "1.74.0"
    license = "Unlicense"
    url = "https://github.com/graphitemaster/incbin"
    git_url = "https://github.com/graphitemaster/incbin.git"
    git_commit = "8cefe46d5380bf5ae4b4d87832d811f6692aae44"
    git_dir = "incbin"
    no_copy_source = True
    description = "Include binary files in C/C++"
    topics = ("embedded binary resources",)
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    options = {
        "header_only": [True, False],
    }
    default_options = {
        "header_only": True,
    }

    exports_sources = ["CMakeLists.txt"]

    _cmake = None

    def source(self):
        git = tools.Git(folder=self.git_dir)
        git.clone(self.git_url)
        git.checkout(self.git_commit)
        dst = os.path.join(self.source_folder, self.git_dir)
        shutil.copy("CMakeLists.txt", dst)

    def configure(self):
        if self.options.header_only:
            self.settings.clear()

    def _configure_cmake(self):
        if self._cmake:
            return self._cmake
        self._cmake = CMake(self)
        self._cmake.definitions["INCBIN_HEADER_ONLY"] = self.options.header_only
        self._cmake.configure(source_folder=self.git_dir)
        return self._cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
