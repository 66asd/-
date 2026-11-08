# Automatic code backups

Project namespace `dds-saopin` uses the user-approved destination https://github.com/66asd/-.git.
Include this file, .gitignore, README.md, Sweep.ioc, Core C sources and headers, Tests/check_firmware.py,
and MDK-ARM/Sweep.uvprojx, MDK-ARM/Sweep.uvoptx and the startup assembly.
Exclude vendor Drivers (regenerable with CubeMX), local debugger settings and build output.
Follow the git-code-backup skill for isolated sequential before/after snapshots.
