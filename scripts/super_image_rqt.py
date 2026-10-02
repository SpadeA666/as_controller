#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""在一个 rqt 窗口里左右并排显示两路 YOLO 检测图像。

用 rqt_gui 的 Image View 插件 (rqt_image_view/ImageView) 建两个视图:
    plugin 0 -> /yolov8/camera_1/detection_image/compressed
    plugin 1 -> /yolov8/camera_2/detection_image/compressed

rqt 在 PyQt5 5.14 下无法从 perspective 文件导入窗口大小/停靠布局
(QByteArray 不接受 str), 所以这两件事放在这里由代码完成:
启动后等两个视图都就绪, 把主窗口放大并做左右分屏, 省去手动拖拽。

话题等插件配置仍在 config/super_image.perspective 里, 便于单独修改。
"""
import os
import sys

# 启动时的窗口大小与位置 (屏幕为 2560x1600)
# 想再改大小/位置, 改下面四个数字即可
WINDOW_X = 200
WINDOW_Y = 100
WINDOW_W = 960
WINDOW_H = 450

# 最多等待多少毫秒让两个 Image View 视图加载完成
WAIT_TIMEOUT_MS = 30000
POLL_INTERVAL_MS = 300


def _log(message):
    """打印状态信息。

    ROS/rqt 会把 sys.stderr 换掉, 这里直接用原始流, 保证 roslaunch 的
    output="screen" 下能看到。
    """
    try:
        sys.__stderr__.write(message + '\n')
        sys.__stderr__.flush()
    except Exception:  # noqa: BLE001 - 日志失败不能影响界面
        pass


def _perspective_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'config', 'super_image.perspective')


from python_qt_binding.QtCore import QTimer, Qt  # noqa: E402
from python_qt_binding.QtWidgets import QDockWidget  # noqa: E402
from rqt_gui.main import Main as RqtGuiMain  # noqa: E402


class Main(RqtGuiMain):
    """rqt_gui 启动器: 加载双视图 perspective, 然后自动放大并左右分屏。"""

    def __init__(self, filename=None, ros_pack=None, settings_filename='rqt_gui'):
        super(Main, self).__init__(
            filename=filename, ros_pack=ros_pack, settings_filename=settings_filename)
        self._arranged = False
        self._waited_ms = 0

    def create_application(self, argv):
        app = super(Main, self).create_application(argv)
        QTimer.singleShot(POLL_INTERVAL_MS, lambda: self._arrange_when_ready(app))
        return app

    def _find_main_window(self, app):
        for widget in app.topLevelWidgets():
            if widget.objectName() == 'MainWindow':
                return widget
        return None

    def _find_image_views(self, main_window):
        if main_window is None:
            return []
        docks = [dock for dock in main_window.findChildren(QDockWidget)
                 if dock.objectName().startswith('rqt_image_view__ImageView__')]
        docks.sort(key=lambda dock: dock.objectName())
        return docks

    def _arrange_when_ready(self, app):
        main_window = self._find_main_window(app)
        docks = self._find_image_views(main_window)

        if self._arranged:
            return
        if len(docks) < 2:
            self._waited_ms += POLL_INTERVAL_MS
            if self._waited_ms < WAIT_TIMEOUT_MS:
                QTimer.singleShot(POLL_INTERVAL_MS, lambda: self._arrange_when_ready(app))
            else:
                _log('[super_image] 警告: 只找到 %d 个 Image View 视图, 跳过自动布局'
                     % len(docks))
            return

        self._arranged = True
        left, right = docks[0], docks[1]

        main_window.setWindowTitle('YOLO 检测图像 (camera_1 | camera_2)')
        for dock in (left, right):
            dock.setFloating(False)
            dock.show()
            # 放开插件自带的最小尺寸限制, 否则窗口宽度会被顶到约 1028 而到不了设定值
            inner = dock.widget()
            if inner is not None:
                inner.setMinimumSize(1, 1)
        # 左右分屏: 两幅图各占一半
        main_window.splitDockWidget(left, right, Qt.Horizontal)
        main_window.resizeDocks([left, right], [1, 1], Qt.Horizontal)
        main_window.resize(WINDOW_W, WINDOW_H)
        main_window.move(WINDOW_X, WINDOW_Y)

        _log('[super_image] 已就绪: 窗口 %dx%d, 左右两个视图 %s / %s'
             % (WINDOW_W, WINDOW_H, left.objectName(), right.objectName()))


def main():
    argv = ['rqt_gui', '--perspective-file', _perspective_path()]
    # 额外参数原样透传 (例如 --verbose), 方便排查问题
    argv += sys.argv[1:]
    return Main().main(argv)


if __name__ == '__main__':
    sys.exit(main())
