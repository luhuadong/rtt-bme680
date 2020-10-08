from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add bme680 src files.
if GetDepend('PKG_USING_BME680'):
    src += Glob('BME680_driver/bme680.c')
    src += Glob('src/sensor_bosch_bme680.c')

if GetDepend('PKG_USING_BME680_SAMPLE'):
    src += Glob('examples/sensor_bme680_sample.c')

# add bme680 include path.
path  = [cwd + '/inc', cwd + '/BME680_driver']

# add src and include to group.
group = DefineGroup('bme680', src, depend = ['PKG_USING_BME680'], CPPPATH = path)

Return('group')