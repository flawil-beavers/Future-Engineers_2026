"""Replace only upstream VL53L4CX I/O bodies in a generated build source.

Uses the installed PlatformIO build middleware. Never edits library packages
or their caches; generated sources stay under ignored local_workspace and
objects remain in the default .pio/build environment directory. The original
vendor license and all non-transport functions are preserved.
"""
from pathlib import Path


def replace_transport(source):
    for operation,argument in (('Read','NumByteToRead'),('Write','NumByteToWrite')):
        signature=f'VL53L4CX_Error VL53L4CX::VL53L4CX_I2C{operation}('
        if source.count(signature)!=1:
            raise RuntimeError('Unexpected VL53L4CX transport version; review bounded I/O adapter')
        start=source.index(signature)
        opening=source.index('{',start)
        end=opening+1;depth=1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        declaration=source[start:opening]
        for name in ('DeviceAddr','RegisterAddr','pBuffer',argument):
            if name not in declaration:
                raise RuntimeError('VL53L4CX I/O signature changed; review bounded I/O adapter')
        body='\n{\n  return BoundedTofTransport::'+operation.lower()+(
            f'(*dev_i2c,DeviceAddr,RegisterAddr,pBuffer,{argument},DEFAULT_I2C_BUFFER_LEN);\n}}')
        source=source[:opening]+body+source[end:]
    include='#include "vl53l4cx_class.h"'
    if source.count(include)!=1:
        raise RuntimeError('VL53L4CX header include changed; review bounded I/O adapter')
    return source.replace(include,include+'\n#include "bounded_tof_transport.h"',1)


def install(build_env):
    def bounded_io(library_env,node):
        original=Path(node.srcnode().get_abspath())
        generated=(Path(library_env.subst('$PROJECT_DIR'))/'local_workspace'/
                   'generated_tof'/library_env.subst('$PIOENV')/'vl53l4cx_class.cpp')
        patched=replace_transport(original.read_text())
        generated.parent.mkdir(parents=True,exist_ok=True)
        if not generated.exists() or generated.read_text()!=patched:
            generated.write_text(patched)
        print('Using bounded VL53L4CX transport for '+library_env.subst('$PIOENV'))
        return library_env.Object(target=node.get_abspath()+'.o',source=str(generated))
    build_env.AddBuildMiddleware(bounded_io,'*STM32duino VL53L4CX*vl53l4cx_class.cpp')


try:
    Import
except NameError:
    pass
else:
    Import('env')
    install(env)
