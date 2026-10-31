"""Finite remote deployment test for the K230 Wi-Fi dev agent."""

import time


print("REMOTE_EXEC_TEST_BEGIN")
for index in range(5):
    print("REMOTE_EXEC_TEST_TICK", index)
    time.sleep_ms(200)
print("REMOTE_EXEC_TEST_OK")
