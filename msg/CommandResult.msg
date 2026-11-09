# Outcome of a command, returned in the response of every commanding service.

uint8 ACCEPTED=0
uint8 REFUSED_LIMIT=1           # outside the vehicle's limits (speed, depth, altitude, region, rate)
uint8 REFUSED_NO_CONTROL=2      # the client does not hold control
uint8 REFUSED_STALE=3           # sequence not new, or stamp older than the freshness limit
uint8 REFUSED_NO_TOKEN=4        # no valid authentication block
uint8 REFUSED_UNSUPPORTED=5     # the vehicle's family or version does not have this command or field
uint8 REFUSED_INVALID=6         # content invalid (unknown manoeuvre kind, missing field, text too long for the medium)
uint8 FAILED=7                  # accepted but not carried out

uint8 code

# Why a command was refused or failed; names the manoeuvre or the field when the content is at fault. Empty when accepted.
string reason
