# Authentication block carried by every request that commands a vehicle (plans, payloads, text out, external
# control, abort). Read-only requests (QueryPlan) carry none.
#
# The session token is never sent in a message: the client obtains it from a host-local channel and only uses it to
# compute `mac`. A host that runs without an authenticated path (the allow-list, the arming file and the key are
# configured on the host) accepts an empty block; a host with the path enabled refuses a request whose block fails.
#
# mac = keyed hash (HMAC) over `sequence`, `stamp` and the request content, computed with the session key.

# Strictly increasing per client session; a request whose sequence is not new is refused as stale.
uint64 sequence

# Time the client signed the request, on the clock the host runs (the simulation clock on a simulated host).
# A request older than the host's freshness limit is refused as stale.
builtin_interfaces/Time stamp

# Keyed hash, raw bytes. Empty: no authentication offered.
uint8[] mac
