"""
build a tmuxp file for launching autonomy
"""

import yaml
import os
import copy

base_tmuxp = {
    "session_name": "autonomy",
    "windows":[
        {
            "window_name": "autonomy",
            "layout": "tiled",
            "panes":[
                {
                }
            ]
        }
    ]
}

def make_shell_command(base_cmd, args={}, delay=0.):
    """
    make a tmuxp shell command from a base command, args and a delay
    Args:
        base_cmd: The base command ot launch (i.e. roslaunch torch_mpc mppi.launch)
        args: The args to append to the base command (dict of <arg_name: arg_val>)
        delay: The time in s to wait before executing the command
    """
    out = {
        "shell_command":[
            "cd ~/physics_atv_ws",
            "source ~/physics_atv_ws/devel/setup.bash",
        ]
    }

    out["shell_command"].append("sleep {}".format(delay))
    cmd = base_cmd
    if args is not None:
        for k,v in args.items():
            cmd += " {}:={}".format(k,v)
    out["shell_command"].append(cmd)
    return out

def generate_tmuxp_config(config, metadata):
    """
    create a tmuxp launch from config, metadata dicts
    """
    out_config = copy.deepcopy(base_tmuxp)

    launches = metadata['pre']['launch']

    for launch_k, launch_v in launches.items():
        assert launch_k in config['launch'].keys(), "Got launch {} but could not find corresponding key in config.yaml launch".format(launch_k)

        shell_cmd = make_shell_command(
            base_cmd=config['launch'][launch_k]['launch_cmd'],
            args=launch_v['args'],
            delay=config['launch'][launch_k]['launch_delay']
        )

        out_config['windows'][0]['panes'].append(shell_cmd)
        
    return out_config
