import os, sys, time, hashlib

SP = sys.argv[1]; NET = sys.argv[2]


def one_run(out, port):
    import traci
    traci.start(['sumo-gui', '-n', NET, '--step-length', '0.001', '--no-step-log', 'true',
                 '--start', 'true', '--quit-on-end', 'true', '--window-size', '1148,646',
                 '--gui-settings-file', SP + '/demo_view.xml'], port=port)
    for k in range(8):
        traci.simulationStep(k * 0.001)
    traci.gui.screenshot('View #0', out, width=1148, height=646)
    traci.simulationStep(0.009)
    traci.simulationStep(0.010)
    time.sleep(0.6)
    traci.close()
    time.sleep(1.5)


print('run 1...', flush=True)
one_run(SP + '/D1.png', 9130)
print('D1', os.path.exists(SP + '/D1.png'), flush=True)
time.sleep(3)
print('run 2...', flush=True)
one_run(SP + '/D2.png', 9131)
print('D2', os.path.exists(SP + '/D2.png'), flush=True)


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()


if os.path.exists(SP + '/D1.png') and os.path.exists(SP + '/D2.png'):
    h1, h2 = md5(SP + '/D1.png'), md5(SP + '/D2.png')
    print('RESULT D1 md5', h1, flush=True)
    print('RESULT D2 md5', h2, flush=True)
    print('RESULT BYTE_IDENTICAL', h1 == h2, flush=True)
    import numpy as np, cv2
    a = cv2.imread(SP + '/D1.png').astype(int); b = cv2.imread(SP + '/D2.png').astype(int)
    print('RESULT PIXEL_MAXDIFF', int(np.abs(a - b).max()), flush=True)
else:
    print('RESULT FALLO_ALGUN_RUN', flush=True)
print('DONE', flush=True)
