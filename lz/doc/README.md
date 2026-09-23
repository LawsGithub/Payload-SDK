# 航点文件参考

本目录是**开发期参考资料**，不参与编译、不打包进 dpk。

## KMZ 是什么

`.kmz` 就是一个 **zip**（`file` 命令会告诉你 `Zip archive data`），
里面固定两个 XML：

```text
wpmz/template.kml     航线模板：任务配置、坐标模式、每个航点的定义
wpmz/waylines.wpml    可执行航线：飞机实际执行的那份
```

以官方样例（`samples/sample_c/module_sample/waypoint_v3/waypoint_file/
waypoint_v3_test_file.kmz`，3856 字节）为例，解出来是
`template.kml` 19814 字节 + `waylines.wpml` 27262 字节。

### 每个航点长什么样

```xml
<Placemark>
  <Point><coordinates>113.943123779504,22.57749398646</coordinates></Point>
  <wpml:index>0</wpml:index>
  <wpml:ellipsoidHeight>100</wpml:ellipsoidHeight>
  <wpml:height>100</wpml:height>
  <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
  <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>
  <wpml:headingleParam>…</wpml:headingleParam>
  <wpml:actionGroup>
    <wpml:actionGroupId>0</wpml:actionGroupId>
    <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
    <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
    <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
    <wpml:actionTrigger>
      <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
    </wpml:actionTrigger>
    <wpml:action>
      <wpml:actionId>0</wpml:actionId>
      <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
      <wpml:actionActuatorFuncParam>
        <wpml:fileSuffix>航点1</wpml:fileSuffix>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:actionActuatorFuncParam>
    </wpml:action>
  </wpml:actionGroup>
</Placemark>
```

`useGlobalHeight` / `useGlobalSpeed` / `useGlobalHeadingParam` / `useGlobalTurnParam`
这几个开关决定该点是用全局值还是自己的值 —— **本项目要逐点不同的云台角，
所以这些开关不能全开**，否则逐点设定会被全局值覆盖。

### 任务级配置（`template.kml` 顶部）

```xml
<wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
<wpml:finishAction>gotoFirstWaypoint</wpml:finishAction>
<wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
<wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
<wpml:takeOffSecurityHeight>20</wpml:takeOffSecurityHeight>
<wpml:globalTransitionalSpeed>15</wpml:globalTransitionalSpeed>
<wpml:droneInfo>          <wpml:droneEnumValue>77</wpml:droneEnumValue>  <!-- 77 = M3E -->
<wpml:payloadInfo>        <wpml:payloadEnumValue>66</wpml:payloadEnumValue><!-- 66 = M3E 相机 -->
```

坐标模式：`coordinateMode=WGS84` + `heightMode=relativeToStartPoint` +
`positioningType=GPS`。

⚠️ **`droneEnumValue` / `payloadEnumValue` 必须与实际机型负载匹配** ——
对照 `psdk_lib/include/dji_typedef.h` 的 `E_DjiAircraftType` / `E_DjiCameraType`。
样例里是 77/66（M3E），妙算3 挂载时要按实际情况改。

本项目实际下发 **99 / 89**（`DJI_AIRCRAFT_TYPE_M4T` / `DJI_CAMERA_TYPE_M4T`，
子类型 1 / 0），定义在 `LzWpml_DefaultIdentity()`。**是否被飞机接受尚未实测** ——
现场判据见 [`ONDEVICE-CHECKLIST.md`](ONDEVICE-CHECKLIST.md) §3.7。

## 样例源码自己怎么说

`test_waypoint_v3.c` 里有一句官方注释：

> Attention: suggest use the exported kmz file by DJI pilot. If use this test
> file, you need set the longitude as 113.94255, latitude as 22.57765 on DJI
> Assistant 2 simulator

**官方推荐用 DJI Pilot 导出的 KMZ 做模板** —— 这提示了一条低成本路径：
先用 Pilot 手工画一条绕飞航线导出，拿它当骨架来对照/生成，
比从零拼 XML 稳妥得多。
