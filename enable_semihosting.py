Import("env")

env.ProcessUnFlags("--specs=nosys.specs")
env.ProcessUnFlags("-lnosys")

# Add semihosting feature
env.Append(
    LINKFLAGS=["--specs=rdimon.specs"],
    LIBS=["rdimon"]
)

def skip_syscalls(node):
    file_name = env.GetProjectOption("syscalls_file", "syscalls.c")

    if file_name in node.name:
        print("Note: not building {0} as semihosting is enabled!".format(node.name))
        return None

    return node

env.AddBuildMiddleware(skip_syscalls)

## In order to use this, add the following line to your platformio.ini in an appropriate environment:
##
## extra_scripts = pre:enable_semihosting.py
##
## Note: if the c-lib sys calls implementation source file is on a different file than "syscalls.c",
## add the following option to the same environment where this script is added:
##
## syscalls_file = "the_file.c"