#!/usr/bin/python
# -*- coding: utf-8 -*-
__author__ = 'Fabien Poussin'
__version__ = '0.1'

from os.path import dirname, abspath
from argparse import ArgumentParser
import re

parser = ArgumentParser(description='Update ChibiOS halconf and mcuconf from STM32CubeMX project files.')
parser.add_argument('cube', type=str, help='STM32CubeMX project file')
parser.add_argument('cfg', type=str, help='Chibios config files folder')

# Always enable
ALWAYS = ('PAL', 'EXTI')

# In case IPs don't match hal names, or if you want to override (ie: use SERIAL instead of UART driver)
HAL_TRANSLATE = (
    ('USB', 'USB_OTG_FS'),
    ('USB', 'USB_OTG_HS'),
    ('SDC', 'SDMMC'),
    ('TRNG', 'RNG'),
    ('WSPI', 'QUADSPI'),
    ('WDG', 'IWDG'),
    ('UART', 'USART')
)

DRIVER_TRANSLATE = (
    ('SDC', 'SDMMC'),
    ('SERIAL', r'U(S)?ART'),
    ('UART', r'U(S)?ART'),
    ('USB', 'OTG'),
    ('PWM', 'TIM'),
    ('ICU', 'TIM'),
    ('GPT', 'TIM'),
    ('WDG', 'IWDG'),
    ('WSPI', 'QUADSPI'),
    ('RNG', 'RNG')
)

RCC_TRANSLATE = (
    ('HPRE', 'HCLK'),
    ('PPRE1', 'APB1CLKDivider'),
    ('PPRE2', 'APB2CLKDivider'),
    ('SW', 'SYSCLKSource'),
    ('SDMMC1SEL', 'SDMMCClockSelection'),
    ('PLLM_VALUE', 'PLLM'),
    ('PLLN_VALUE', 'PLLN'),
    ('PLLQ_VALUE', 'PLLQ'),
    ('PLLSAIN_VALUE', 'PLLSAIN'),
    ('PLLSAIR_VALUE', 'PLLSAIR'),
    ('MCO1SEL', 'RCC_MCO1Source'),
    ('MCO2SEL', 'RCC_MCO2Source'),
    ('PLLI2SN_VALUE', 'PLLI2SN'),
)

def translate_hal(ip):
    for h in HAL_TRANSLATE:
        if re.search(h[1], ip, re.M):
            return h[0]
    return ip

def translate_driver(ip):
    for d in DRIVER_TRANSLATE:
        if re.search(d[0], ip, re.M):
            return d[1]
    return ip

def set_boolean_define(line, match, name, value):
    if name in line and re.search(match, line, re.M):
        if value == True:
            line = line.replace('FALSE', 'TRUE')
        else:
            line = line.replace('TRUE', 'FALSE')
        print(line.strip())
    return line

def get_hal_devices(source):
    out = []
    for line in source:
        if '#define HAL_USE_' in line:
            l = line.split(' ')
            dev = ('_').join(l[1].split('_')[2:])
            if dev not in out:
                out.append(dev)
    return out

def get_enabled_drivers(source, hal_devices):
    out = {}
    for line in source:
        if line.startswith('Mcu.IP'):
            ip_only = re.search(r"^Mcu.IP\d+=((I2C|[A-Z]+_?)+)(\d)?", line) # Extract ip name separated from periph number
            if ip_only:
                dev = translate_hal(ip_only.group(1)) # periph name
                dev_num = ip_only.group(3) # periph number
                if dev in hal_devices:
                    if dev not in out.keys():
                        out[dev] = []
                    if dev_num:
                        out[dev].append(dev_num)
    return out

def get_rcc(source):
    out = {}

    return out

def update_hal(source, drivers):
    match = '#define HAL_USE_'
    for i in range(len(source)):
        line = source[i]
        if line.startswith(match):
            if "TRUE" in line:
                source[i] = line.replace('TRUE', 'FALSE')

        for d in drivers:
            source[i] = set_boolean_define(source[i], match, d, True)

    return source

def update_drivers(source, drivers):
    for i in range(len(source)):
        line = source[i]
        if '_USE_' in line:
            if 'TRUE' in line:
                source[i] = line.replace('TRUE', 'FALSE')

        for driver, instances in drivers.items():
            if instances:
                for inst in instances:
                    periph = translate_driver(driver)
                    match = 'STM32_{0}_USE_{1}{2}'.format(driver, periph, inst)
                    source[i] = set_boolean_define(source[i], match, driver, True)
            else:
                periph = translate_driver(driver)
                match = 'STM32_{0}_USE_{1}'.format(driver, periph)
                source[i] = set_boolean_define(source[i], match, driver, True)

    return source

def update_rcc(source, rcc):
    # TODO
    return source

if __name__ == '__main__':
    args = parser.parse_args()
    cur_path = dirname(abspath(__file__))

    halconf_path = args.cfg + '/halconf.h'
    mcuconf_path = args.cfg + '/mcuconf.h'

    with open(args.cube, 'r') as f:
        project = f.readlines()

    with open(halconf_path, 'r') as f:
        halconf = f.readlines()

    with open(mcuconf_path, 'r') as f:
        mcuconf = f.readlines()

    hal_devices = get_hal_devices(halconf)
    enabled_drivers = get_enabled_drivers(project, hal_devices)
    rcc = get_rcc(project)

    for a in ALWAYS:
        enabled_drivers[a] = []

# Update and save halconf
    halconf = update_hal(halconf, enabled_drivers)
    with open(halconf_path, 'w') as f:
        f.write("".join(halconf))

# Update and save mcuconf drivers
    mcuconf = update_drivers(mcuconf, enabled_drivers)
    mcuconf = update_rcc(mcuconf, rcc)
    with open(mcuconf_path, 'w') as f:
        f.write("".join(mcuconf))
