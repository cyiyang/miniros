# reminder 功能包

提醒小车在目标板更新时前往目标板查看新的任务。

## 服务定义

Server 端：负责目标板识别的节点。
Client 端：`reminder`.

请求包含字段：

- need_to_see: bool 类型。

返回包含字段：
- see_done: bool 类型。

返回允许有延迟。也允许真的看完之后再返回。
