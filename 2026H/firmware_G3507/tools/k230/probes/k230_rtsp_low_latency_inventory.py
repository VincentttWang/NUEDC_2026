"""Check CanMV APIs required by the low-latency RTSP data path."""

import multimedia as mm

from media.vencoder import StreamData


stream_data = StreamData()
server = mm.rtsp_server()

print("RTSP_LOW_LATENCY_INVENTORY_BEGIN")
print("STREAM_HAS_PHY_ADDR", hasattr(stream_data, "phy_addr"))
print(
    "RTSP_HAS_PHY_SEND",
    hasattr(server, "rtspserver_sendvideodata_byphyaddr"),
)
print("RTSP_LOW_LATENCY_INVENTORY_END")
