import time


last_frame_time = 0
time_step = 0.01

for _ in range(100):
    time_spent = time.time() - last_frame_time
    print("TIME SPENT: ", time_spent)
    last_frame_time = time.time()
    print("LAST TIME FRAME: ", last_frame_time)
    action_repeat=1
    time_to_sleep = action_repeat * time_step - time_spent
    print("Time to Sleep: ", time_to_sleep)
    
    if time_to_sleep > 0:
        time.sleep(time_to_sleep)