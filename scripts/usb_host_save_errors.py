"""Preserve USB mass-storage errors in the generated M7 library source."""
from pathlib import Path


def replace_once(source, before, after):
    if source.count(before) != 1:
        raise RuntimeError('Arduino_USBHostMbed5 has changed; review the USB save corrections')
    return source.replace(before, after, 1)


def fix_mass_storage(source):
    source = replace_once(
        source,
        '    return SCSITransfer(NULL, 6, DEVICE_TO_HOST, 0, 0);',
        '    uint8_t cmd[6] = {}; // TEST UNIT READY can return CHECK CONDITION.\n'
        '    return SCSITransfer(cmd, sizeof(cmd), DEVICE_TO_HOST, 0, 0);',
    )
    source = replace_once(
        source,
        '        return SCSIRequestSense();',
        '        // REQUEST SENSE reports the cause, not success of the failed command.\n'
        '        SCSIRequestSense();\n'
        '        return -1;',
    )
    return replace_once(
        source,
        '    disk_init = 1;\n    return readCapacity();',
        '    const int status = readCapacity();\n'
        '    disk_init = status == 0 && blockSize > 0 && blockCount > 0;\n'
        '    return disk_init ? 0 : -1;',
    )


def fix_host(source):
    return replace_once(
        source,
        '    volatile HCTD * td = ed->getNextTD();\n'
        '    if (td == NULL) {\n'
        '        return USB_TYPE_ERROR;\n',
        '    volatile HCTD * td = ed->getNextTD();\n'
        '    if (td == NULL) {\n'
        '        td_mutex.unlock();\n'
        '        return USB_TYPE_ERROR;\n',
    )


def install(build_env):
    def corrected_source(library_env, node):
        original = Path(node.srcnode().get_abspath())
        transform = fix_host if original.name == 'USBHost.cpp' else fix_mass_storage
        generated = (Path(library_env.subst('$PROJECT_DIR')) / 'local_workspace' /
                     'generated_usb_host' / library_env.subst('$PIOENV') / original.name)
        patched = transform(original.read_text())
        # The generated source is no longer next to its vendor header.
        header = original.relative_to(original.parents[1]).with_suffix('.h').as_posix()
        patched = replace_once(patched, f'#include "{original.stem}.h"', f'#include "{header}"')
        generated.parent.mkdir(parents=True, exist_ok=True)
        if not generated.exists() or generated.read_text() != patched:
            generated.write_text(patched)
        print('Using USB save error handling in ' + original.name)
        return library_env.Object(target=node.get_abspath() + '.o', source=str(generated))

    build_env.AddBuildMiddleware(corrected_source, '*Arduino_USBHostMbed5*USBHost.cpp')
    build_env.AddBuildMiddleware(corrected_source, '*Arduino_USBHostMbed5*USBHostMSD.cpp')


try:
    Import
except NameError:
    pass
else:
    Import('env')
    install(env)
