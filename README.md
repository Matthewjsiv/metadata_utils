# metadata_utils
Framework for automatically launching modules and generating useful metadata for rosbags before, during, and after data collection.

NOTE: Current working version is actually the 20221005_striest branch, I need to merge it back in.

## How to Use
### Set Up
To set up your field experiment, there are (hopefully) up to 2 files you need to change:

#### config.yaml
Config.yaml contains all the shortcuts that streamline setting up your actual field test yaml. You can add your own launch file shortcuts by following the examples that currently exist there.
You can also add more sensors and the topics they publish. This is just to help the post-processor figure out which sensors are present in your bags.

If you need to make modifications based on specific things you're running, you can change the way they're processed by adding code in generate_tmuxp_config.py

#### Field Test Yaml File
You can now set up your own field test yaml, following any of the examples in the configs folder, such as template.yaml or matthew_experiments.yaml. 

Data_folder is the high-level directory to save runs in, and the experiment_name is the subfolder within data_folder that it will save runs to. To add modules to be launched, you just need to add their shorthand names (defined by you in config.yaml) in the launch section, and you can include args there as well (I haven't fully tested the args but they have worked so far). Additionally, you should add context keywords, and other information like the current weather conditions. This will make it easier to filter data later on.

I also made a module called live_notes that opens a terminal window and lets you type in notes as you are testing. It will save them into a txt file in your run folder, with each note being timestamped.

### At Test Time
To run your experiment, run: 
`python3 collect.py --name configs/template.yaml --descriptor test_1`
where template.yaml will be replaced by your yaml. Descriptor is optional, if you don't use it, your run information will be saved in a folder named by time, placed in data_folder/experiment_name. If you do use it, it will just add it as a prefix to the time and then save it.

### After Field Test
After your finish all the runs for an experiment, you can run `python3 post.py --folder /path/to/experiment_name` and it will run some post-processing scripts that will add to the yaml files for all your runs information such as bag duration and what sensors were present.
