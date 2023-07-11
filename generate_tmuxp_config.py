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

def make_shell_command(base_cmd, args={}, delay=0., on_atv = False):
    """
    make a tmuxp shell command from a base command, args and a delay
    Args:
        base_cmd: The base command ot launch (i.e. roslaunch torch_mpc mppi.launch)
        args: The args to append to the base command (dict of <arg_name: arg_val>)
        delay: The time in s to wait before executing the command
    """
    if on_atv:
        out = {
            "shell_command":[
                "sshp",
                "cd ~/physics_atv_ws",
                "source ~/physics_atv_ws/devel/setup.bash",
            ]
        }
    else:
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

def generate_tmuxp_config(config, metadata, now=None, descriptor=None):
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
            delay=config['launch'][launch_k]['launch_delay'],
            on_atv='on_atv' in config['launch'][launch_k]
        )

        out_config['windows'][0]['panes'].append(shell_cmd)

    other = metadata['pre']['other']
    #
    for k,v in other.items():
        # if "rosbag record" in config['launch'][launch_k]['launch_cmd']:
        #     out = {
        #         "shell_command":[
        #             "cd ~/physics_atv_ws",
        #             "source ~/physics_atv_ws/devel/setup.bash",
        #         ]
        #     }
        #
        #     out["shell_command"].append("sleep {}".format(config['launch'][launch_k]['launch_delay']))
        #     print(config['launch'][launch_k]['launch_cmd'])
        #     cmd = config['launch'][launch_k]['launch_cmd'] + " -O " + os.path.join(metadata['data_folder'], metadata['experiment_name'],now,descriptor) + ".bag"
        #     out["shell_command"].append(cmd)
        #     shell_cmd = out

        if k == 'record':
            out = {
                    "shell_command":[
                        "cd ~/physics_atv_ws",
                        "source ~/physics_atv_ws/devel/setup.bash",
                    ]
                }
            out["shell_command"].append("sleep {}".format(config['other'][k]['launch_delay']))
            cmd = config['other'][k]['launch_cmd'] + " -O " + os.path.join(metadata['data_folder'], metadata['experiment_name'],now,descriptor) + ".bag"
            out["shell_command"].append(cmd)
            shell_cmd = out
            out_config['windows'][0]['panes'].append(shell_cmd)
        if k == 'live_notes':
            out = {
                    "shell_command":[
                        # "cd ~/physics_atv_ws",
                        "source ~/physics_atv_ws/devel/setup.bash",
                    ]
                }
            cmd = config['other'][k]['launch_cmd'] + " -f " + os.path.join(metadata['data_folder'], metadata['experiment_name'],now,descriptor) + ".txt"
            out["shell_command"].append(cmd)
            shell_cmd = out
            out_config['windows'][0]['panes'].append(shell_cmd)


    return out_config
