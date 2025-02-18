script_folder="/root/workspace/ros2/src/my_vins/conan_cmake"
echo "echo Restoring environment" > "$script_folder/deactivate_conanrunenv-debug-x86_64.sh"
for v in LD_LIBRARY_PATH DYLD_LIBRARY_PATH PATH
do
    is_defined="true"
    value=$(printenv $v) || is_defined="" || true
    if [ -n "$value" ] || [ -n "$is_defined" ]
    then
        echo export "$v='$value'" >> "$script_folder/deactivate_conanrunenv-debug-x86_64.sh"
    else
        echo unset $v >> "$script_folder/deactivate_conanrunenv-debug-x86_64.sh"
    fi
done


export LD_LIBRARY_PATH="/root/.conan2/p/b/ceres7559e225d63a0/p/lib:/root/.conan2/p/b/gtsam1b524b6f7db93/p/lib:/root/.conan2/p/b/boostb9624c9ff04a8/p/lib:/root/.conan2/p/b/zlib1b8f81f2e5881/p/lib:/root/.conan2/p/b/bzip25c07710f128fc/p/lib:/root/.conan2/p/b/libba2b4b7bc0f0381/p/lib:/root/.conan2/p/b/onetbfe249136fcbde/p/lib:/root/.conan2/p/b/hwlocac4c0a1154c1a/p/lib:/root/.conan2/p/b/metisd77a7b51d0f09/p/lib:/root/.conan2/p/b/gklib14be21712bd5b/p/lib:/root/.conan2/p/b/fmt1a579931855d1/p/lib:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="/root/.conan2/p/b/ceres7559e225d63a0/p/lib:/root/.conan2/p/b/gtsam1b524b6f7db93/p/lib:/root/.conan2/p/b/boostb9624c9ff04a8/p/lib:/root/.conan2/p/b/zlib1b8f81f2e5881/p/lib:/root/.conan2/p/b/bzip25c07710f128fc/p/lib:/root/.conan2/p/b/libba2b4b7bc0f0381/p/lib:/root/.conan2/p/b/onetbfe249136fcbde/p/lib:/root/.conan2/p/b/hwlocac4c0a1154c1a/p/lib:/root/.conan2/p/b/metisd77a7b51d0f09/p/lib:/root/.conan2/p/b/gklib14be21712bd5b/p/lib:/root/.conan2/p/b/fmt1a579931855d1/p/lib:$DYLD_LIBRARY_PATH"
export PATH="/root/.conan2/p/b/bzip25c07710f128fc/p/bin:$PATH"