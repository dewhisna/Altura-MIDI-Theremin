Import("env")
import subprocess

GIT_VERSION = "g" + subprocess.check_output(["git", "describe", "--tags", "--always", "--dirty=-dirty"]).strip()

env.Append(CCFLAGS=["-DGIT_VERSION="+GIT_VERSION])
