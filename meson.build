project('qemu', ['c', 'cpp'], meson_version: '>=0.52.999')

kconfig = import('unstable-kconfig')
config_host = kconfig.load(meson.current_build_dir() / 'config-host.mak')

add_project_arguments(config_host['CFLAGS'].split(),
                      language: ['c', 'objc'])
add_project_arguments(config_host['QEMU_CFLAGS'].split(),
                      language: 'c')
add_project_link_arguments(config_host['LDFLAGS'].split(),
                           language: ['c', 'cpp', 'objc'])
add_project_link_arguments(config_host['QEMU_LDFLAGS'].split(),
                           language: ['c', 'cpp', 'objc'])
add_project_arguments(config_host['QEMU_INCLUDES'].split(),
                      language: 'c')

configure_file(input: files('scripts/ninjatool.py'),
               output: 'ninjatool',
               configuration: config_host)
