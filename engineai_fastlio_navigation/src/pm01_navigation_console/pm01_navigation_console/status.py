"""Render concrete diagnostic states and freshness, never health gate booleans."""
import math


def number(value):
    return isinstance(value, (int, float)) and math.isfinite(value) and value >= 0


def detail(snapshot):
    lines = []
    motion = snapshot.get('motion', {})
    age = snapshot.get('motion_receive_age_sec')
    stale = number(age) and age > 2.5
    prefix = '上次输出报告（{:.1f} 秒前）'.format(age) if stale else '本项目输出'
    lines.append(prefix+'：{}　输出进程：{}　发布者：{}'.format(
        motion.get('state', '未收到'), motion.get('output_process_running', '未知'), motion.get('output_publishers', '未知')))
    lines.append('输出已撤下：{}　目标取消：{}　物理控制归属：未知'.format(
        '当前未确认' if stale else motion.get('output_detached', '未知'), motion.get('goal_cancel', '未请求')))
    lines.append('Nav2 接口：{}　地图有空闲区域：{}'.format(
        snapshot.get('action_server_ready', False), snapshot.get('map_has_free_cells', False)))
    lines.append('取消按钮请求：'+snapshot.get('cancel_request', '未请求'))
    for topic, name in [('/lio/status', 'LIO'), ('/pm01/map_update_status', '地图'),
                        ('/pm01/localization_status', '旧图定位')]:
        entry = snapshot.get('diagnostics', {}).get(topic, {})
        age = entry.get('receive_age_sec')
        age_text = '{:.1f} 秒前'.format(age) if number(age) else '未收到'
        text = '{}：{}（{}）'.format(name, entry.get('message', '未收到'), age_text)
        last = entry.get('values', {}).get('last_successful_update_age_sec')
        if topic == '/pm01/map_update_status':
            text += '　距成功更新：'+('{:.1f} 秒'.format(last+(age or 0)) if number(last) else '尚未获得')
        lines.append(text)
    return '\n'.join(lines)
