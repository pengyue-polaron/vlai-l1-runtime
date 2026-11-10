# VLAI L1 Runtime

Jetson source root: /home/nvidia/vlai-l1-runtime.
This is the only default AI instruction document. Consult README.md or relevant
code when needed; do not routinely load historical notes or archived guides.

1. **Task-level authority.** Complete the authorized task without per-step approval.
   Start/next-step means physical preparation is complete; do not repeat generic
   readiness/injury questions. Ask only for critical missing facts, scope expansion
   or irreversible effects not already authorized.
2. **Reuse work and check only what changed.** Reuse healthy services, existing
   setup, completed results and still-valid checks. Do not restart or repeat them
   for reassurance. Test the affected behavior; document edits need only link/diff
   checks. Runtime feedback freshness remains a live check, not a cached result.
3. **Keep control coherent.** Use one owner per device, correct arm/units/order,
   valid targets and fresh feedback, working motor protections and a verified
   fault/shutdown path. Normal transitions and normal finish must not unload the arm.
4. **Scope requirements to the task.** Check only needed devices and inputs.
   Unused peripherals, future APIs and old experiments are not general gates.
   Parameters belong in config/code and can be changed within the authorized task.
5. **Recover using logs.** Bounded software retries/restarts are allowed without
   disturbing live motor ownership. Do not blindly re-enable motors after faults.
   Diagnose with relevant logs; do not create extra evidence packages, hash
   manifests, audit receipts or experiment reports by default.
6. **Use Git and preserve work.** Manage source, configuration and documents in Git;
   do not create backup copies, archives or snapshots by default. Preserve unrelated
   edits and valuable data/weights; authorized cleanup needs no per-file approval.
   Keep instructions current and concise, without appended historical timelines.
