# 测试记录

以下截图来自项目调试过程，串口工具显示 COM17，配置为 115200、8 数据位、奇校验、1 停止位、无流控。接收工具可能把一条设备输出分成多个显示片段。

## 连续记录与跨扇区

[查看截图：时间设置和跨 409 条记录](screenshots/recording-sector-boundary.jpg)

截图显示 `RTC Set OK`、`Saved #409`、`Saved #410` 和 `Records: 411`。这证明该次运行已跨过首个扇区的 409 条边界；没有证明最大容量全部写满或长期耐久性。

## 导出

- [导出开始](screenshots/export-start.jpg)：发送 `D` 后出现 CSV 表头和记录。
- [导出结束](screenshots/export-end.jpg)：末尾出现 `END`，随后继续出现新的 `Saved` 记录。

## 重启后恢复

[查看截图：重启后恢复记录数](screenshots/restart-recovery.jpg)

截图中重启前可见 `Saved #460`，重新启动后显示 `Stored DS18B20 records: 460`，随后继续写入。用户另行反馈断电重插后可正常工作；截图本身未测定断电时间内的时钟误差。

## 尚需按实物验证

- USB／电池独立供电与切换、RTC 后备电池保持时间。
- 功能按键与指示灯的现场动作。
- 低温、高温端测量误差及完整容量或长期运行测试。
