#!/usr/bin/python
import sys
from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c
if sys.version_info >= (3, 0):
    import configparser
else:
    import ConfigParser as configparser


class Motor():
    def __init__(self, name='motor', thread='base_thread', eqep='eQEP0',
                 eqepScale=2797.0,
                 pwmDown='hpg.pwmgen.00.out.00',
                 pwmUp='hpg.pwmgen.00.out.01',
                 enableDown='bb_gpio.p9.out-15',
                 enableUp='bb_gpio.p9.out-17'):

        sigPgain = hal.newsig('%s-pgain' % name, hal.HAL_FLOAT)
        sigIgain = hal.newsig('%s-igain' % name, hal.HAL_FLOAT)
        sigDgain = hal.newsig('%s-dgain' % name, hal.HAL_FLOAT)
        sigCmdVel = hal.newsig('%s-cmd-vel' % name, hal.HAL_FLOAT)
        sigPwmIn = hal.newsig('%s-pwm-in' % name, hal.HAL_FLOAT)
        sigPos = hal.newsig('%s-pos' % name, hal.HAL_FLOAT)
        sigVel = hal.newsig('%s-vel' % name, hal.HAL_FLOAT)
        sigAcc = hal.newsig('%s-acc' % name, hal.HAL_FLOAT)
        sigUp = hal.newsig('%s-pwm-up' % name, hal.HAL_FLOAT)
        sigDown = hal.newsig('%s-pwm-down' % name, hal.HAL_FLOAT)
        sigEnable = hal.newsig('%s-enable' % name, hal.HAL_BIT)
        sigPwmEn = hal.newsig('%s-pwm-enable' % name, hal.HAL_BIT)

        sigTuneStart = hal.newsig('%s-tune-start' % name, hal.HAL_BIT)
        sigTuneMode = hal.newsig('%s-tune-mode' % name, hal.HAL_BIT)

        # 43.7:1 gear
        # encoder resolution of 64 counts per revolution of the motor shaft,
        # 2797 counts per revolution of the gearboxs output shaft.

        # for eQEP0.position in shaft revs:
        hal.Pin('%s.position-scale' % eqep).set(eqepScale)
        hal.Pin('%s.capture-prescaler' % eqep).set(5)
        hal.Pin('%s.min-speed-estimate' % eqep).set(0.001)

        # feed into PID
        sigPos.link('%s.position' % eqep)
        # for UI feedback
        sigVel.link('%s.velocity' % eqep)

        # ddt for accel
        ddt = rt.newinst('ddt', 'ddt.%s-acc' % name)
        hal.addf(ddt.name, thread)
        ddt.pin('in').link(sigVel)
        ddt.pin('out').link(sigAcc)

        # PID
        pid = rt.newinst('at_pid', 'pid.%s-vel' % name)
        hal.addf('%s.do-pid-calcs' % pid.name, thread)
        pid.pin('maxoutput').set(1.0)  # set maxout to prevent windup effect
        pid.pin('Pgain').link(sigPgain)
        pid.pin('Igain').link(sigIgain)
        pid.pin('Dgain').link(sigDgain)
        pid.pin('command').link(sigCmdVel)
        pid.pin('output').link(sigPwmIn)
        pid.pin('feedback').link(sigVel)
        pid.pin('enable').link(sigEnable)

        # auto tuning
        pid.pin('tuneCycles').set(200)
        pid.pin('tuneEffort').set(0.15)
        pid.pin('tuneMode').link(sigTuneMode)
        pid.pin('tuneStart').link(sigTuneStart)

        # automatically start auto tuning when switched to tune mode
        timedelay = rt.newinst('timedelay', 'timedelay.%s' % sigTuneStart.name)
        hal.addf(timedelay.name, thread)
        timedelay.pin('in').link(sigTuneMode)
        timedelay.pin('on-delay').set(0.1)
        timedelay.pin('off-delay').set(0.0)

        # convert out singnal to IO
        outToIo = rt.newinst('out_to_io', 'out-to-io.%s' % sigTuneStart.name)
        hal.addf(outToIo.name, thread)
        timedelay.pin('out').link(outToIo.pin('in-bit'))
        outToIo.pin('out-bit').link(sigTuneStart)

        # reset the tune mode to false once tuning is finished
        reset = rt.newinst('reset', 'reset.%s' % sigTuneMode.name)
        hal.addf(reset.name, thread)
        reset.pin('out-bit').link(sigTuneMode)
        reset.pin('reset-bit').set(False)
        reset.pin('trigger').link(sigTuneStart)
        reset.pin('rising').set(False)
        reset.pin('falling').set(True)

        # hbridge
        hbridge = rt.newinst('hbridge', 'hbridge.%s' % name)
        hal.addf(hbridge.name, thread)
        hbridge.pin('up').link(sigUp)
        hbridge.pin('down').link(sigDown)
        hbridge.pin('enable').link(sigEnable)
        hbridge.pin('enable-out').link(sigPwmEn)
        hbridge.pin('command').link(sigPwmIn)

        # PWM signals
        hal.Pin('%s.value' % pwmUp).link(sigUp)
        hal.Pin('%s.value' % pwmDown).link(sigDown)

        # Enable
        hal.Pin(enableUp).link(sigPwmEn)
        hal.Pin(enableDown).link(sigPwmEn)
        hal.Pin('%s.enable' % pwmUp).link(sigPwmEn)
        hal.Pin('%s.enable' % pwmDown).link(sigPwmEn)

        # prevent pid runup if disabled
        sigEnable.set(True)

        # storage
        hal.Pin('storage.%s.pgain' % name).link(sigPgain)
        hal.Pin('storage.%s.igain' % name).link(sigIgain)
        hal.Pin('storage.%s.dgain' % name).link(sigDgain)


def setupPosPid(name='pos', thread='base_thread'):
    sigPgain = hal.newsig('%s-pgain' % name, hal.HAL_FLOAT)
    sigIgain = hal.newsig('%s-igain' % name, hal.HAL_FLOAT)
    sigDgain = hal.newsig('%s-dgain' % name, hal.HAL_FLOAT)
    sigVel = hal.newsig('%s-vel' % name, hal.HAL_FLOAT)
    sigFeedback = hal.newsig('%s-feedback' % name, hal.HAL_FLOAT)
    sigOutput = hal.newsig('%s-output' % name, hal.HAL_FLOAT)
    sigCmd = hal.newsig('%s-cmd' % name, hal.HAL_FLOAT)
    sigEnable = hal.newsig('%s-enable' % name, hal.HAL_BIT)

    pid = rt.newinst('pid', 'pid.%s' % name)
    hal.addf('%s.do-pid-calcs' % pid.name, thread)
    pid.pin('maxoutput').set(1.0)  # set maxout to prevent windup effect
    pid.pin('Pgain').link(sigPgain)
    pid.pin('Igain').link(sigIgain)
    pid.pin('Dgain').link(sigDgain)
    pid.pin('feedback-deriv').link(sigVel)
    pid.pin('feedback').link(sigFeedback)
    pid.pin('output').link(sigOutput)
    pid.pin('command').link(sigCmd)
    pid.pin('enable').link(sigEnable)

    kalman = hal.components['kalman']
    kalman.pin('rate').link(sigVel)
    kalman.pin('angle').link(sigFeedback)

    # TODO use output
    # TODO use cmd

    # storage
    hal.Pin('storage.%s.pgain' % name).link(sigPgain)
    hal.Pin('storage.%s.igain' % name).link(sigIgain)
    hal.Pin('storage.%s.dgain' % name).link(sigDgain)

    sigEnable.set(True)


def setupGyro(thread='base_thread'):
    name = 'balance'
    sigReq = hal.newsig('%s-req' % name, hal.HAL_BIT)
    sigAck = hal.newsig('%s-ack' % name, hal.HAL_BIT)
    sigDt = hal.newsig('%s-dt' % name, hal.HAL_FLOAT)
    sigNewAngle = hal.newsig('%s-new-angle' % name, hal.HAL_FLOAT)
    sigNewRate = hal.newsig('%s-new-rate' % name, hal.HAL_FLOAT)

    gyroaccel = hal.loadusr('./hal_gyroaccel', name='gyroaccel',
                            bus_id=1, interval=0.05,
                            wait_name='gyroaccel')
    gyroaccel.pin('req').link(sigReq)
    gyroaccel.pin('ack').link(sigAck)
    gyroaccel.pin('dt').link(sigDt)
    gyroaccel.pin('angle').link(sigNewAngle)
    gyroaccel.pin('rate').link(sigNewRate)
    gyroaccel.pin('invert').set(True)  # invert the output since we mounted the gyro upside down

    kalman = rt.loadrt('kalman', 'names=kalman')
    hal.addf(kalman.name, thread)
    kalman.pin('req').link(sigReq)
    kalman.pin('ack').link(sigAck)
    kalman.pin('dt').link(sigDt)
    kalman.pin('new-angle').link(sigNewAngle)
    kalman.pin('new-rate').link(sigNewRate)


def setupStorage():
    hal.loadusr('hal_storage', name='storage', file='storage.ini',
                autosave=True, wait_name='storage')


def readStorage():
    hal.Pin('storage.read-trigger').set(True)  # trigger read


rt.init_RTAPI()
c.load_ini('hardware.ini')

rt.loadrt('hal_bb_gpio', output_pins='915,917,838,840')
rt.loadrt('hal_arm335xQEP', encoders='eQEP0,eQEP2')
rt.loadrt(c.find('PRUCONF', 'DRIVER'), 'prucode=' + c.find('PRUCONF', 'PRUBIN'), pru=0, num_pwmgens=7, halname='hpg')

# storage
setupStorage()

# pru pwmgens
hal.Pin('hpg.pwmgen.00.pwm_period').set(500000)
# motor left
hal.Pin('hpg.pwmgen.00.out.00.pin').set(911)
hal.Pin('hpg.pwmgen.00.out.01.pin').set(913)
# motor right
hal.Pin('hpg.pwmgen.00.out.02.pin').set(808)
hal.Pin('hpg.pwmgen.00.out.03.pin').set(810)

baseThread = 'base_thread'
rt.newthread(baseThread, 1000000, fp=True)
hal.addf('bb_gpio.read', baseThread)
hal.addf('eqep.update', baseThread)

ml = Motor(name='ml', eqep='eQEP0', eqepScale=-2797.0,
           pwmUp='hpg.pwmgen.00.out.01',
           pwmDown='hpg.pwmgen.00.out.00',
           enableDown='bb_gpio.p9.out-15',
           enableUp='bb_gpio.p9.out-17')
mr = Motor(name='mr', eqep='eQEP2', eqepScale=2797.0,
           pwmUp='hpg.pwmgen.00.out.02',
           pwmDown='hpg.pwmgen.00.out.03',
           enableDown='bb_gpio.p8.out-38',
           enableUp='bb_gpio.p8.out-40')

setupGyro()
setupPosPid()
readStorage()

hal.addf('hpg.update', baseThread)
hal.addf('bb_gpio.write', baseThread)

hal.start_threads()
