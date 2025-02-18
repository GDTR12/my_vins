from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain

class MyPackage(ConanFile):
    name = "mypackage"
    version = "1.0"
    settings = "os", "arch", "compiler", "build_type"
    requires = (
        "ceres-solver/2.2.0",
        "gtsam/4.2",
        "sophus/1.22.10",
        "fmt/10.1.0"
    )
    generators = "CMakeDeps", "CMakeToolchain"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {
        "shared": True,  # 当前包编译为动态库
        "fPIC": True,
        # 强制所有依赖项编译为动态库
        "*:shared": True,  # 关键配置：通配符 * 表示所有依赖项
    }

    def build(self):
        # 执行构建步骤，如果需要在构建过程中自定义操作可以在此添加
        pass

    # def layout(self):
    #     # 设置包的布局
    #     self.folders.build = "build"
    #     self.folders.generators = "build"
    #     self.folders.src = "src"

    def package(self):
        # 如果需要打包，定义打包步骤
        pass

    def configure_cmake(self):
        # 你可以在此处自定义 CMake 相关的配置
        pass
