script_folder="/root/workspace/ros2/src/my_vins/conan_cmake"
echo "echo Restoring environment" > "$script_folder/deactivate_conanrunenv-release-x86_64.sh"
for v in LD_LIBRARY_PATH DYLD_LIBRARY_PATH PATH
do
    is_defined="true"
    value=$(printenv $v) || is_defined="" || true
    if [ -n "$value" ] || [ -n "$is_defined" ]
    then
        echo export "$v='$value'" >> "$script_folder/deactivate_conanrunenv-release-x86_64.sh"
    else
        echo unset $v >> "$script_folder/deactivate_conanrunenv-release-x86_64.sh"
    fi
done


export LD_LIBRARY_PATH="/root/.conan2/p/b/ceresfbb374a6ae484/p/lib:/root/.conan2/p/b/gtsamed2e2c7b741a1/p/lib:/root/.conan2/p/b/boosta3674f0613c88/p/lib:/root/.conan2/p/b/zlib7d4291166ce78/p/lib:/root/.conan2/p/b/bzip2a407bd902e4e6/p/lib:/root/.conan2/p/b/libba401dd34da2b0a/p/lib:/root/.conan2/p/b/onetb71c6afbc2e98f/p/lib:/root/.conan2/p/b/hwlocd1e9d75d1f1c6/p/lib:/root/.conan2/p/b/metis56d01366a7dba/p/lib:/root/.conan2/p/b/gklib959d1aac4a586/p/lib:/root/.conan2/p/b/fmta937b8d71049a/p/lib:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="/root/.conan2/p/b/ceresfbb374a6ae484/p/lib:/root/.conan2/p/b/gtsamed2e2c7b741a1/p/lib:/root/.conan2/p/b/boosta3674f0613c88/p/lib:/root/.conan2/p/b/zlib7d4291166ce78/p/lib:/root/.conan2/p/b/bzip2a407bd902e4e6/p/lib:/root/.conan2/p/b/libba401dd34da2b0a/p/lib:/root/.conan2/p/b/onetb71c6afbc2e98f/p/lib:/root/.conan2/p/b/hwlocd1e9d75d1f1c6/p/lib:/root/.conan2/p/b/metis56d01366a7dba/p/lib:/root/.conan2/p/b/gklib959d1aac4a586/p/lib:/root/.conan2/p/b/fmta937b8d71049a/p/lib:$DYLD_LIBRARY_PATH"
export PATH="/root/.conan2/p/b/bzip2a407bd902e4e6/p/bin:$PATH"