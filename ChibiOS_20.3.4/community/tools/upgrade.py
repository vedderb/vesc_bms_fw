#!/usr/bin/python3
# -*- coding: utf-8 -*-
__author__ = 'Fabien Poussin'
__version__ = '0.1'

import os
import argparse


verbose = False


def find(name, path):
    ret = []
    for root, dirs, files in os.walk(path):
        for f in files:
            if f == name:
                ret.append(os.path.abspath(os.path.join(root, f)))
    return ret


def is_nil(path):
    with open(path, 'r') as f:
        for l in f.readlines()[:100]:
            if l.startswith('#define') and '_CHIBIOS_NIL_CONF_' in l:
                return True
    return False


def get_values(path):
    values = {}
    with open(path, 'r') as f:
        continuation = None
        line_num = 0
        for l in f.readlines():
            if l.startswith('#define') or continuation != None:
                if continuation is None:
                    words = l.split(' ')
                    if len(words) > 2:
                        defname = words[1]
                        defval = ' '.join(words[2:])
                        values[defname] = {'data': [defval], 'pos': line_num, 'offset': 0}
                    else:
                        line_num += 1
                        continue
                else:
                    values[continuation]['data'].append(l)
                # multi line define
                if l.rstrip().endswith('\\') and continuation is None:
                    continuation = defname
                # one line define
                elif not l.rstrip().endswith('\\'):
                    continuation = None
            line_num += 1
    return values


def set_values(from_, to_):
    offset = 0
    for k in from_:
        if k in to_:
            to_[k]['offset'] = offset
            curr_offset = len(from_[k]['data']) - len(to_[k]['data'])
            if curr_offset != 0 and verbose:
                print(k, 'offset:', curr_offset)
            to_[k]['data'] = from_[k]['data']
            offset += curr_offset
        elif verbose:
            print('Missing:', k)


def write_changes(path, source, values):
    with open(source, 'r') as f:
        file_lines = f.readlines()

        lines = {}
        for v in get_values(source).values():
            lines[v['pos']] = len(v['data'])

        # Delete all defines starting from bottom
        for l in reversed(tuple(lines.keys())):
            size = lines[l]
            del file_lines[l:l+size]

        # Find last #endif to add our #include
        if 'halconf.h' in source:
            l_num = 0
            for l in reversed(file_lines):
                l_num += 1
                if l.startswith('#endif'):
                    break
            file_lines.insert(-l_num, '#include "halconf_community.h"\n\n')

        for v in values.keys():
            size = len(values[v]['data'])
            pos = values[v]['pos']
            file_lines.insert(pos, '#define {0} {1}'.format(v, values[v]['data'][0]))
            if not file_lines[pos].endswith('\n'):
                file_lines[pos] += '\n'
            if size > 1:
                for i in range(size)[1:]:
                    file_lines.insert(pos+i, values[v]['data'][i])
                    if not file_lines[pos+i].endswith('\n'):
                        file_lines[pos+i] += '\n'

    with open(path, 'w') as f:
        f.write(''.join(file_lines))


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Project upgrade script for ChibiOS')
    parser.add_argument('-p', '--path', dest='path', help='Where project files are located', required=True)
    parser.add_argument('-c', '--chibios', dest='chpath', help='Where ChibiOS is located', required=True)
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true')
    args = parser.parse_args()

    verbose = args.verbose

    if not os.path.exists(args.chpath):
        print('Invalid Chibios path', args.chpath)
        exit(1)

    new_chconf = os.path.abspath('{}/os/rt/templates/chconf.h'.format(args.chpath))
    new_nilconf = os.path.abspath('{}/os/nil/templates/chconf.h'.format(args.chpath))
    new_halconf = os.path.abspath('{}/os/hal/templates/halconf.h'.format(args.chpath))

    new_chconf_values = get_values(new_chconf)
    new_nilconf_values = get_values(new_nilconf)
    new_halconf_values = get_values(new_halconf)

    chconf_files = find('chconf.h', args.path)
    halconf_files = find('halconf.h', args.path)

    for c in chconf_files:
        values = get_values(c)
        if verbose:
            print(c)
            for k, v in values.items():
                print(k, v)
        if is_nil(c):
            new_format = new_nilconf_values.copy()
            set_values(values, new_format)
            write_changes(c, new_nilconf, new_format)
        else:
            new_format = new_chconf_values.copy()
            set_values(values, new_format)
            write_changes(c, new_chconf, new_format)

    for h in halconf_files:
        values = get_values(h)
        if verbose:
            print(h)
            for k, v in values.items():
                print(k, v)
        new_format = new_halconf_values.copy()

        set_values(values, new_format)
        write_changes(h, new_halconf, new_format)
