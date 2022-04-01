import pandas as pd
# @app.callback([Output("ws", "send"), Output("esp-cmd-params", "data")], [Input("cmd-input-button", "n_clicks")], [State("cmd-input", "value")])
# def parse_cmd(n_clicks, input):
#     CMD_LEN = 12
#     input_list = input.split(' ')

#     if input_list[0] == "SET":
#         param = input_list[1]
#         values = [val for val in input_list[2:]]
#         to_send = param
#         to_send = to_send + "_" + \
#             values[0] if len(values) >= 1 else to_send + "\n"
#         to_send = to_send + "_" + values[1] + \
#             "\n" if len(values) >= 2 else to_send
#         if len(to_send) != CMD_LEN:
#             to_send = to_send.ljust(CMD_LEN)
#             print("to_send: " + to_send + " len: " + str(len(to_send)))
#             return to_send, input_list
#     else:
#         print(input_list)
#         return None, None


# @app.callback(Output("placeholder", "children"), [Input("esp-cmd-params", "data")], [State("esp-set-val-tbl", "data")])
# def populate_cmd_table(input_list, tbl_data):
#     param = input_list[1]
#     values = input_list[2:]
#     return placeholder

def populate_rec_data_table(n, esp_data):
    # TODO: make these in a dict that can be parsed
    HEADER_LOOKUP = {"dt": "dt",
                     "yaw": "yaw",
                     "c_yaw": "c_yaw",
                     "F_ULT": "front_dist",
                     "S_ULT": "side_dist",
                     "BV": "batt_voltage",
                     "gx": "gyro_x",
                     "gy": "gyro_y",
                     "gz": "gyro_z"}
    if esp_data == "Connected to ESP Websocket":
        print(esp_data)
        return no_update
    print(esp_data)
    # TODO: Add separator
    try:
        data_list = esp_data.split(' ')
    except AttributeError:
        return no_update
    print(data_list)
    headers = []
    for item in data_list:
        for key, val in HEADER_LOOKUP.items():
            if item == key:
                headers.append(val)

        # headers.append(
        #     val for key, val in HEADER_LOOKUP.items() if item == key)

    # HEADERS = [HEADER_LOOKUP[item] for key in HEADER_LOOKUP.keys() if item == key for item in data_list]
    print(headers)
    data_list = [item for i, item in enumerate(data_list) if i % 2 != 0]
    print(data_list)
    # HEADER_LIST = [val if item == HEADER_LOOKUP[key] for key, val in HEADER_LOOKUP for item in data_list]
    # HEADER_LIST = ["dt", "yaw", "c_yaw", "front_dist"]
    DATA_LEN = len(headers)
    df = pd.DataFrame(data={"Parameters": headers,
                      "Values": data_list[:DATA_LEN]})
    print(df)
    return df.to_dict('records')


if __name__ == '__main__':
    print(populate_rec_data_table(0, "dt 1 yaw 1 c_yaw 1 F_ULT 1"))
    print(populate_rec_data_table(
        0, "dt 203 yaw 0.23 c_yaw 9.304 F_ULT 1 gx 320.0"))
    print(populate_rec_data_table(0, "S_ULT 30.2"))
    # print(populate_rec_data_table(0, "ds"))
    # parse_cmd("SET PID_kp 12")
    # parse_cmd("SET PID_ki 4")
    # parse_cmd("SET adec 030 2.15")
    # parse_cmd("SET stop")
    # parse_cmd("GET PID_ki 4")
    # parse_cmd("BLARGH")
