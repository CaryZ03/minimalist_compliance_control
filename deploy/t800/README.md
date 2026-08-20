# T800 minimal deployment

Deploy only the headless MCC right-arm runtime to a selected T800 Orin:

```bash
./deploy/t800/sync.sh t800e
```

The target is `/home/ubuntu/mcc-t800`. Dependencies are isolated under that
directory and the vendor ROS environment is reused without modifying `/app` or
`/apps`.

Run a bounded, non-publishing validation first:

```bash
ssh -t t800e '/home/ubuntu/mcc-t800/run.sh shadow --duration 10'
```

`zero_replay` and `command` create a joint-override publisher and therefore
require the explicit `--allow-output` flag. Do not enable them until the robot
is in `lower_body_balance` and shadow validation has passed:

```bash
ssh -t t800e '/home/ubuntu/mcc-t800/run.sh zero_replay --allow-output'
ssh -t t800e '/home/ubuntu/mcc-t800/run.sh command --allow-output'
```
