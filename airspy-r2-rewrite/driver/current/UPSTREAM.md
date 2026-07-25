# Imported baseline

This source tree was imported from Airspy's `airspyone_host` repository at:

```text
fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0
```

It includes the local prepared-start compatibility change:

- try `RECEIVER_MODE_ARMED`;
- submit host transfers;
- enter `RECEIVER_MODE_RX`;
- fall back to the legacy one-step RX request when older firmware stalls the
  armed value.

No transfer geometry or public API change is part of that compatibility patch.
