# Fatal signal handling note

The legacy `airspy_rx` tool installs its graceful-stop callback for `SIGSEGV`,
`SIGILL`, `SIGFPE`, and `SIGABRT` as well as `SIGINT`/`SIGTERM`. The callback
calls `fprintf`, writes a non-`sig_atomic_t` flag, and returns. Returning from a
synchronous `SIGSEGV` resumes the faulting instruction and creates the observed
infinite `Caught signal 11` loop, hiding the original crash location.

The Rust driver/tool rebuild should use ordinary structured shutdown for normal
API termination and handle only user-request signals such as `SIGINT` and
`SIGTERM`. Fatal process signals must retain their default disposition so the
first fault produces a useful crash or sanitizer report. Stream start, finite
capture completion, cancellation, callback retirement, and device close must
also have one explicit ownership/state transition each.
