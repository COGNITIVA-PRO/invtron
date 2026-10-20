Import("env")

from SCons.Script import Default


program = env.get("PIOMAINPROG")
if program is None:
    raise RuntimeError("PlatformIO did not create the firmware program target")

compile_only = env.AddCustomTarget(
    name="compile-only",
    dependencies=program[0].sources,
    actions=None,
    title="Compile Sources Only",
    description="Compile source files without linking the firmware",
)

Default(compile_only)
