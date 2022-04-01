from dash import Dash, html, dcc, Input, Output, State, dash_table, no_update
from dash_extensions import WebSocket
import pandas as pd
import json
import dash_bootstrap_components as dbc

PORT = 8080  # Port to listen on (non-privileged ports are > 1023)
# HOST = f"ws://0.0.0.0:{PORT}"
HOST = f"ws://192.168.4.1:81"

app = Dash(__name__, prevent_initial_callbacks=True,
           external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = html.Div(children=[
    html.H1(children='MTE380 Group 4 Dashboard: LIGHTNING MCQUEEN'),
    html.P("Please type your command input and hit submit:"),
    html.Div(dcc.Input(id="cmd-input", autoComplete="off")),
    html.Button('Submit', id='cmd-input-button', n_clicks=0),
    html.Br(),
    html.Br(),
    dcc.Store(id='esp-store', storage_type='memory'),
    dcc.Store(id='esp-cmd-store', storage_type='memory'),
    dcc.Store(id='esp-cmd-params', storage_type='memory'),
    WebSocket(url=HOST, id="ws"),
    html.Div([
        # style={'textAlign': 'center'}
        # html.Label("Lightning McQueen Params and Values"),
        dash_table.DataTable(columns=[{"name": ["Lightning McQueen Params and Values", "Parameters"], "id": "Parameters"}, {
             "name": ["Lightning McQueen Params and Values", "Values"], "id": "Values"}], id='esp-output-tbl', merge_duplicate_headers=True, style_cell={'textAlign': 'center'}),
        dcc.Interval(
            id='interval-component',
            interval=1*1000,  # in milliseconds
            n_intervals=0
        )
    ]),
    html.Br(),
    html.Br(),
    html.Div([
        # html.Label("Lightning McQueen Command Set Values",
        #    style={'textAlign': 'center'}),
        dash_table.DataTable(columns=[{"name": ["Lightning McQueen Command Set Values", "Parameters"], "id": "Parameters"}, {
             "name": ["Lightning McQueen Command Set Values", "Values"], "id": "Values"}], id='esp-set-val-tbl', merge_duplicate_headers=True, style_cell={'textAlign': 'center'}),
        dcc.Interval(
            id='interval-component2',
            interval=1*1000,  # in milliseconds
            n_intervals=0
        )
    ]),
    html.P(id='placeholder'),
    html.Div(id='temp-output')
])

# @app.callback([Output("ws", "send"), Output("esp-cmd-store", "data")], [Input("cmd-input-button", "n_clicks")], [State("cmd-input", "value")])


@app.callback(Output("ws", "send"), [Input("cmd-input-button", "n_clicks")], [State("cmd-input", "value")])
# @app.callback(Output("ws", "send"), [Input("cmd-input", "value")])
def send(n_clicks, value):
    return value


# Update div using websocket.
# receive = 'function(msg){return "Got Response from Websocket" + msg["data"];}'
receive = 'function(msg){return msg["data"];}'
app.clientside_callback(receive, Output("esp-store", "data"),  # Output("ws-output", "children"),
                        [Input("ws", "message")])  # Output("esp-store", "data"),

# receive = 'function(msg){return "Got Response from Websocket" + msg["data"];}'
# app.clientside_callback(receive, Output("temp-output", "children"),  # Output("ws-output", "children"),
#                         [Input("ws", "message")])  # Output("esp-store", "data"),


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


@app.callback(Output("placeholder", "children"), [Input("esp-cmd-params", "data")], [State("esp-set-val-tbl", "data")])
def populate_cmd_table(input_list, tbl_data):
    param = input_list[1]
    values = input_list[2:]
    return placeholder


# [Input("interval-component", "n_intervals")],
@app.callback(Output("esp-output-tbl", "data"), [Input("interval-component", "n_intervals")], [State("esp-store", "data")])
def populate_rec_data_table(n, esp_data):
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
    # print(esp_data)
    # TODO: Add separator
    try:
        data_list = esp_data.split(' ')
    except AttributeError:
        return no_update
    # print(data_list)
    headers = []
    for item in data_list:
        for key, val in HEADER_LOOKUP.items():
            if item == key:
                headers.append(val)
    # print(headers)
    data_list = [item for i, item in enumerate(data_list) if i % 2 != 0]
    # print(data_list)
    DATA_LEN = len(headers)
    df = pd.DataFrame(data={"Parameters": headers,
                      "Values": data_list[:DATA_LEN]})
    print(df)
    return df.to_dict('records')

    # # TODO: make these in a dict that can be parsed
    # HEADER_LOOKUP = {"dt": "dt",
    #                  "yaw": "yaw",
    #                  "c_yaw": "c_yaw",
    #                  "F_ULT": "front_dist",
    #                  "S_ULT": "side_dist",
    #                  "BV": "batt_voltage",
    #                  "gx": "gyro_x",
    #                  "gy": "gyro_y",
    #                  "gx": "gyro_z"}
    # if esp_data == "Connected to ESP Websocket":
    #     print(esp_data)
    #     return no_update
    # print(esp_data)
    # # TODO: Add separator
    # try:
    #     data_list = esp_data.split(' ')
    # except AttributeError:
    #     return no_update

    # headers = []
    # for item in data_list:
    #     headers.append(HEADER_LOOKUP[item]
    #                    for key in HEADER_LOOKUP.keys() if item == key)
    # # HEADER_LIST = [val if item == HEADER_LOOKUP[key] for key, val in HEADER_LOOKUP for item in data_list]
    # HEADER_LIST = ["dt", "yaw", "c_yaw", "front_dist", ]
    # DATA_LEN = len(HEADER_LIST)
    # # print(HEADER_LIST)
    # # print(data_list)
    # # print(data_list[:DATA_LEN])
    # df = pd.DataFrame(data={"Parameters": HEADER_LIST,
    #                   "Values": data_list[:DATA_LEN]})
    # print(df)
    # return df.to_dict('records')


if __name__ == '__main__':
    app.run_server(debug=True)
