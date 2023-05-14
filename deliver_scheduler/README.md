# deliver-scheduler 功能包

`DestinationMsg.srv` 中信息定义如下。
```
int8 car_no
int8 request_type
int8 request_drug_type
int8 request_deliver_destination
---
int8 drug_location
int8 deliver_destination
```

发送请求的参数含义以及可能取值如表所示。
| 变量名                      | 含义                   | 可能取值                                                                                                                 |
|-----------------------------|------------------------|--------------------------------------------------------------------------------------------------------------------------|
| `car_no`                      | 为多车保留的小车编号   | 任何 `int8`                                                                                                                |
| `request_type`                | 服务请求的类型         | 0: 识别得到新的配送需求      1: 请求下一个配送目标      2: 小车到达取药区，且取得药物      3: 小车到达送药区，并交付药物 |
| `request_drug_type`           | 识别到的药物需求类型   | 当 `request_type = 0` 时，该字段为识别到的新的配送需求药物类型；其中三种药物被编码为 0,1,2. 否则该字段为任意 `int8`                                        |
| `request_deliver_destination` | 识别到的药物配送目的地 | 当 `request_type = 0` 时，该字段为识别到的新的配送目的地；否则该字段为任意 `int8`                                              |

仅当 `request_type = 1` 时，返回值才有意义。 `drug_location` 为需求药物类型，三种药物被编码为 0,1,2. `deliver_destination` 为送药地点。当 `request_type` 不为 1 时，以上两个字段的值均为 -1.

## 依赖
依赖于 `python-statemachine`, 由于小车运行 Python 2.7, 因此只能安装 1.0.1 版本。

使用 `pip install python-statemachine==1.0.0` 安装。

若在 import 时出现报错，提示 `ImportError: No module named typing`, 则需要更新 pip 后安装 typing 库。
