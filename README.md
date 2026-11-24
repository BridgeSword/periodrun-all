# periodRunAll.py: run forkZeroOnBoot and check_back_lidar together

periodRun only accepts one script. `periodRunAll.py` is a small entry script. periodRun points to it, and every cycle it calls `forkZeroOnBoot.py` and then `check_back_lidar.py`.

- Neither original script is changed. Each still reads its own params file (`params/forkZeroOnBoot.json`, etc.).
- If one script fails to load or throws an exception, the other keeps running. The failure is logged and shown as user warning **55902**.
- If one full cycle takes longer than 50 ms, a warning is written to the log.

## Install (per bot)

1. **Back up** `robot.model`, and the current scripts folder `scripts/module/`.
2. Copy all files in this package into **`scripts/module/userpy/`**, keeping the folder layout:
   - `periodRunAll.py`
   - `forkZeroOnBoot.py` (V1.1.0, same logic as the one already shared; only the on-screen messages and comments are translated to English)
   - `check_back_lidar.py` (unchanged, copied from bot 7034. On 7034 it was in `scripts/module/`, so it must be in `userpy/` now)
   - `params/forkZeroOnBoot.json` → `scripts/module/userpy/params/forkZeroOnBoot.json`
3. If a bot already has its own `userpy/params/forkZeroOnBoot.json` with site-specific values, keep the bot's file and don't overwrite it.
4. In RoboShop, open the robot model, go to **periodRun → basic → scriptName**, and set it to:
   `userpy/periodRunAll.py`
   Save, then restart Robokit (or reboot the bot).
5. Check:
   - The log shows `periodRunAll: loaded forkZeroOnBoot.py` and `periodRunAll: loaded check_back_lidar.py`.
   - No warning 55902.
   - Fork zero on boot works as before.
   - Sensor check works: temporarily disable one sensor in the model, and error 53920 should appear. Then enable it again.

## To roll back

Set periodRun scriptName back to `userpy/forkZeroOnBoot.py`, or to the old check_back_lidar path, and restart Robokit.

## Notes

- Both scripts write to the same status text (`setInfo`), so the info line on the screen shows whichever script wrote last. Errors and warnings are not affected: forkZeroOnBoot uses 53945 / 53946 / 55900 / 55901, check_back_lidar uses 53920, and periodRunAll uses 55902.
- To add another periodRun script later, put it in the same folder and add its file name to the `SCRIPTS` list at the top of `periodRunAll.py`.
- check_back_lidar.py reads the whole robot model every 100 cycles, which takes about 100 ms on the bot. This is the same when it runs alone. If needed, increase `CHECK_EVERY_CYCLES` in check_back_lidar.py (e.g. 1000) to make the check less frequent.
- Tested offline with a simulated robot interface, and on one RIL-AP bot (Robokit 3.4.8): both scripts load and run, the boot zero check reports correctly, and error 53920 is raised when the Back lidar is disabled. Please try one bot at Casey's first before rolling out.
