import os
import json
import re

from basic import ArenaInfoDynamic

Category = ["Exp_coop", "Exp_Imp_coop", "no_Exp_Imp_coop"]
# Category = ["epsilon_multi", "epsilon_standard"]
# Category = ["epsilon"]

def read_dyn_txt(path):
    if not os.path.exists(path):
        return []
    data = []
    with open(path, "r") as f:
        for (i, line) in enumerate(f.readlines()):
            if i == 0:
                continue
            info = ArenaInfoDynamic(line.strip().split(","))
            data.append(info.to_dict())
        
    return data

def save_as_json(data, path):
    with open(path, "w", encoding='utf-8') as f:
        json.dump(data, f, indent=2)

def filter_state(data):
    whole_process = []
    for d in data:
        vehicle_states = [d["time"]]
        for v in d["vehicle_set"]["vehicles"]:
            if v["type"] == "dyn_obstacle":
                continue

            vehicle_states.append({"id": v["id"], "x": v["state"]["x"], "y": v["state"]["y"], 
                                    "angle": v["state"]["angle"], "curvature": v["state"]["curvature"], 
                                    "velocity": v["state"]["velocity"], "acceleration": v["state"]["acceleration"], 
                                    "steer": v["state"]["steer"]})

        whole_process.append(vehicle_states)
    
    return whole_process

def convert_lane_change_txts(root="/home/pc/MLAD/src/results/lane_change_analysis"):
    for categ in Category:
        print(os.path.join(root, categ, "agents_dyn_info.txt"))
        if not os.path.exists(os.path.join(root, categ, "agents_dyn_info.txt")):
            continue

        dict_data = read_dyn_txt(os.path.join(root, categ, "agents_dyn_info.txt"))

        state_data = filter_state(dict_data)

        save_as_json(dict_data, os.path.join(root, categ, categ + "_all_dyn_info.json"))
        save_as_json(state_data, os.path.join(root, categ, categ + "_all_state_info.json"))

def is_valid_flow_control_result(dict_data, state_data):
    # 是否所有CAV都过线，是否所有CAV的速度有变化
    v_num = len(state_data[-1])-1
    agent_pass = [False] * v_num
    agent_ctrl = [False] * v_num
    for states in state_data:
        for i in range(v_num):
            if agent_pass[i] == False and states[i+1]["x"] > 500:
                agent_pass[i] = True

            if agent_ctrl[i] == False and states[i+1]["velocity"] != 20:
                agent_ctrl[i] = True

    for i in range(v_num):
        if agent_pass[i] == False or agent_ctrl[i] == False:
            return False
    return True

def convert_flow_control_txts(root="/home/pc/MLAD/src/results/flow_control_analysis"):
    for categ in Category:
        idx = 0
        print(os.path.join(root, categ, "agents_dyn_info_raw"))
        files = os.listdir(os.path.join(root, categ, "agents_dyn_info_raw"))
        files = sorted(files)
        for txt_file in files:
            if not os.path.isfile(os.path.join(root, categ, "agents_dyn_info_raw", txt_file)):
                continue
            if not txt_file.endswith(".txt"):
                continue
            dict_data = read_dyn_txt(os.path.join(root, categ, "agents_dyn_info_raw", txt_file))
            state_data = filter_state(dict_data)
            if not is_valid_flow_control_result(dict_data, state_data):
                print("invalid result")
                continue
            
            save_as_json(dict_data, os.path.join(root, categ, "agents_dyn_info_raw", str(idx) + "_all_dyn_info.json"))
            
            if not os.path.exists(os.path.join(root, categ, "agents_state_info")):
                os.mkdir(os.path.join(root, categ, "agents_state_info"))
            save_as_json(state_data, os.path.join(root, categ, "agents_state_info", str(idx) + "_all_state_info.json"))
            idx+=1

def read_eudm_log(path):
    if not os.path.exists(path):
        return []
    data = []
    single_vehicle = {}
    with open(path, "r") as f:
        all_lines = list(f.readlines())
        last_line = all_lines[-1].strip()
        if not last_line.endswith("behavior reply confirm. Write back to smm."):
            return []
        for line in all_lines:
            log_line = line.strip()
            if log_line.find("[Eudm]******************** RUN START: ") != -1:
                start_idx = log_line.find("******************** RUN START: ")
                single_vehicle["time stamp"] = float(log_line[start_idx:].split(" ")[3])
                vid_idx = log_line.find("id:", start_idx) + 3
                vid_idx_end = log_line.find("******************", vid_idx)
                single_vehicle["id"] = int(log_line[vid_idx:vid_idx_end])

            elif log_line.find("[Eudm][Manager] desired_action.lat: ") != -1:
                single_vehicle["desired lat action"] = log_line.split(" desired_action.lat: ")[1]

            elif log_line.find("[Eudm]Prepare time cost ") != -1:
                time_cost = log_line.split("Prepare time cost ")[1]
                single_vehicle["prepare time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]RunOnce time cost ") != -1:
                time_cost = log_line.split("RunOnce time cost ")[1]
                single_vehicle["RunOnce time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]Sum & Reselect time cost ") != -1:
                time_cost = log_line.split("Sum & Reselect time cost ")[1]
                single_vehicle["Sum & Reselect time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]Sum of time: ") != -1:
                time_cost = log_line.split("Sum of time: ")[1]
                end_idx = time_cost.find(" ms, diff")
                single_vehicle["sum of time"] = float(time_cost[:end_idx])

            elif log_line.find("[Eudm][Output]Reselected ") != -1:
                single_vehicle["selected policy"] = log_line[log_line.find(">[")+2 : log_line.find("]<")]

            elif log_line.find("[Eudm]******************** RUN FINISH: ") != -1:
                data.append(single_vehicle)
                single_vehicle = {}

    return data

def read_epsilon_eudm_log(path):
    if not os.path.exists(path):
        return []
    data = []
    single_vehicle = {}
    with open(path, "r") as f:
        for line in f.readlines():
            log_line = line.strip()
            if log_line.find("[Eudm]******************** RUN START: ") != -1:
                start_idx = log_line.find("******************** RUN START: ")
                single_vehicle["time stamp"] = float(log_line[start_idx:].split(" ")[3].split("******************")[0])

            elif log_line.find("[Eudm][Manager] desired_action.lat: ") != -1:
                single_vehicle["desired lat action"] = log_line.split(" desired_action.lat: ")[1]

            elif log_line.find("[Eudm]Prepare time cost ") != -1:
                time_cost = log_line.split("Prepare time cost ")[1]
                single_vehicle["prepare time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]RunOnce time cost ") != -1:
                time_cost = log_line.split("RunOnce time cost ")[1]
                single_vehicle["RunOnce time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]Sum & Reselect time cost ") != -1:
                time_cost = log_line.split("Sum & Reselect time cost ")[1]
                single_vehicle["Sum & Reselect time cost"] = float(time_cost[:-3])

            elif log_line.find("[Eudm]Sum of time: ") != -1:
                time_cost = log_line.split("Sum of time: ")[1]
                end_idx = time_cost.find(" ms, diff")
                single_vehicle["sum of time"] = float(time_cost[:end_idx])

            elif log_line.find("[Eudm][Output]Reselected ") != -1:
                single_vehicle["selected policy"] = log_line[log_line.find(">[")+2 : log_line.find("]<")]

            elif log_line.find("[Eudm]******************** RUN FINISH: ") != -1:
                data.append(single_vehicle)
                single_vehicle = {}
    
    return data

def read_ssc_log(path):
    if not os.path.exists(path):
        return []
    data = []
    single_vehicle = {}
    with open(path, "r") as f:
        for line in f.readlines():
            log_line = line.strip()

            if log_line.find("[Ssc]******************** RUNONCE START: ") != -1:
                start_idx = log_line.find("******************** RUNONCE START: ")
                single_vehicle["time stamp"] = float(log_line[start_idx:].split(" ")[3])
            
            elif log_line.find("[Summary]Mpdm time cost: ") != -1:
                time_cost = log_line.split("[Summary]Mpdm time cost: ")[1]
                single_vehicle["mpdm time cost"] = float(time_cost[:-4])

            elif log_line.find("[Ssc]Sum of time: ") != -1:
                time_cost = log_line.split("Sum of time: ")[1]
                end_idx = time_cost.find(" ms, diff")
                single_vehicle["sum of time"] = float(time_cost[:end_idx])

            elif log_line.find("[Ssc]******************** RUNONCE FINISH: ") != -1:
                data.append(single_vehicle)
                single_vehicle = {}

    return data

def convert_lane_change_logs(root="/home/pc/MLAD/src/results/lane_change_analysis"):
    for categ in Category:
        if not os.path.exists(os.path.join(root, categ, "eudm")):
            continue

        for file_name in os.listdir(os.path.join(root, categ, "eudm")):
            if file_name.startswith("info_"):
                eudm_dict_data = read_eudm_log(os.path.join(root, categ, "eudm", file_name))
        if len(eudm_dict_data) == 0:
            continue

        save_as_json(eudm_dict_data, os.path.join(root, categ + "_eudm_log_info.json"))

        for i in range(4):
            for file_name in os.listdir(os.path.join(root, categ, "ssc_{}".format(i))):
                if file_name.endswith(".INFO"):
                    continue
                ssc_dict_data = read_ssc_log(os.path.join(root, categ, "ssc_{}".format(i), file_name))
            
            save_as_json(ssc_dict_data, os.path.join(root, categ + "_ssc_{}_log_info.json".format(i)))

def convert_flow_control_logs(root="/home/pc/MLAD/src/results/flow_control_analysis", num=7):
    for categ in Category:
        idx = 0
        print(os.path.join(root, categ, "eudm"))
        files = os.listdir(os.path.join(root, categ, "eudm"))
        files = sorted(files)
        for txt_file in files:
            if not os.path.isfile(os.path.join(root, categ, "eudm", txt_file)):
                continue
            if not txt_file.startswith("info_"):
                continue
            eudm_dict_data = read_eudm_log(os.path.join(root, categ, "eudm", txt_file))
            if len(eudm_dict_data) == 0:
                continue

            if not os.path.exists(os.path.join(root, categ, "eudm_log_info")):
                os.mkdir(os.path.join(root, categ, "eudm_log_info"))
            save_as_json(eudm_dict_data, os.path.join(root, categ, "eudm_log_info", str(idx) + "_eudm_log_info.json"))
            idx+=1
        
        for i in range(num):
            idx = 0
            print(os.path.join(root, categ, "ssc_{}".format(i)))
            files = os.listdir(os.path.join(root, categ, "ssc_{}".format(i)))
            files = sorted(files)
            for txt_file in files:
                if txt_file.endswith(".INFO"):
                    continue
                ssc_dict_data = read_ssc_log(os.path.join(root, categ, "ssc_{}".format(i), txt_file))

                if not os.path.exists(os.path.join(root, categ, "ssc_{}_log_info".format(i))):
                    os.mkdir(os.path.join(root, categ, "ssc_{}_log_info").format(i))
                save_as_json(ssc_dict_data, os.path.join(root, categ, "ssc_{}_log_info".format(i), str(idx) + "_ssc_log_info.json"))
                idx+=1

# def convert_epsilon_flow_control_logs(root="/home/pc/MLAD/src/results/flow_control_analysis"):
#     for categ in ["epsilon"]:
#         print(os.path.join(root, categ, "eudm"))
#         subfolders = os.listdir(os.path.join(root, categ, "eudm"))
#         subfolders = sorted(subfolders)
#         times = 0
#         for subf in subfolders:
#             idx = 0
#             files = os.listdir(os.path.join(root, categ, "eudm", subf))
#             files = sorted(files)
#             for file_name in files:
#                 if not os.path.isfile(os.path.join(root, categ, "eudm", subf, file_name)):
#                     continue
#                 if file_name.endswith(".INFO"):
#                     continue

#                 eudm_dict_data = read_epsilon_eudm_log(os.path.join(root, categ, "eudm", subf, file_name))

#                 if not os.path.exists(os.path.join(root, categ, "eudm_{}_log_info".format(idx))):
#                     os.mkdir(os.path.join(root, categ, "eudm_{}_log_info".format(idx)))

#                 save_as_json(eudm_dict_data, os.path.join(root, categ, "eudm_{}_log_info".format(idx), str(times) + "_eudm_log_info.json"))
                
#                 ssc_dict_data = read_ssc_log(os.path.join(root, categ, "eudm", subf, file_name))

#                 if not os.path.exists(os.path.join(root, categ, "ssc_{}_log_info".format(idx))):
#                     os.mkdir(os.path.join(root, categ, "ssc_{}_log_info").format(idx))
#                 save_as_json(ssc_dict_data, os.path.join(root, categ, "ssc_{}_log_info".format(idx), str(times) + "_ssc_log_info.json"))
                
#                 idx+=1
#             times += 1

def convert_epsilon_flow_control_logs(root="/home/pc/MLAD/src/results/flow_control_analysis"):
    for categ in ["epsilon"]:
        print(os.path.join(root, categ, "eudm"))
        files = os.listdir(os.path.join(root, categ, "eudm"))
        files = sorted(files)
        times = {}
        idx = 0
        for file_name in files:
            if not os.path.isfile(os.path.join(root, categ, "eudm", file_name)):
                continue
            if file_name.endswith(".INFO"):
                continue

            eudm_dict_data = read_epsilon_eudm_log(os.path.join(root, categ, "eudm", file_name))
            idx = int(file_name.split("_")[0])
            if idx in times:
                times[idx] += 1
            else:
                times[idx] = 0
            if not os.path.exists(os.path.join(root, categ, "eudm_{}_log_info".format(idx))):
                os.mkdir(os.path.join(root, categ, "eudm_{}_log_info".format(idx)))

            save_as_json(eudm_dict_data, os.path.join(root, categ, "eudm_{}_log_info".format(idx), str(times[idx]) + "_eudm_log_info.json"))
            
            ssc_dict_data = read_ssc_log(os.path.join(root, categ, "eudm", file_name))

            if not os.path.exists(os.path.join(root, categ, "ssc_{}_log_info".format(idx))):
                os.mkdir(os.path.join(root, categ, "ssc_{}_log_info").format(idx))
            save_as_json(ssc_dict_data, os.path.join(root, categ, "ssc_{}_log_info".format(idx), str(times[idx]) + "_ssc_log_info.json"))
            

def convert_lane_change_logs(root="/home/pc/MLAD/src/results/lane_change_analysis"):
    for categ in ["epsilon_multi", "epsilon_standard"]:
        if not os.path.exists(os.path.join(root, categ, "eudm")):
            continue
        idx = 0
        files = os.listdir(os.path.join(root, categ, "eudm"))
        files = sorted(files)
        for file_name in files:
            if not file_name.startswith(".INFO"):
                eudm_dict_data = read_epsilon_eudm_log(os.path.join(root, categ, "eudm", file_name))
                ssc_dict_data = read_ssc_log(os.path.join(root, categ, "eudm", file_name))
        
                save_as_json(eudm_dict_data, os.path.join(root, categ, categ + "_eudm_{}_log_info.json".format(idx)))
                save_as_json(ssc_dict_data, os.path.join(root, categ, categ + "_ssc_{}_log_info.json".format(idx)))
                idx += 1
# convert_lane_change_txts()
# convert_flow_control_txts("/home/pc/MLAD/src/results/flow_control_mix_intent")
# convert_lane_change_logs()
convert_flow_control_logs("/home/pc/MLAD/src/results/flow_control_mix_intent", 5)
# convert_epsilon_flow_control_logs("/home/pc/MLAD/src/results/flow_control_forward_intent")
# convert_lane_change_logs()