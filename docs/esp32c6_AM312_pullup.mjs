import {occupancy, battery, numeric} from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    // v2 = Arduino sketch (deep sleep), v3 = ESP-IDF port (Zigbee light sleep).
    // The Zigbee model is identical between them - same endpoints, clusters and
    // attributes - so one converter serves both and a device still running the
    // old firmware keeps working with this file.
    zigbeeModel: ['AM312_Presence_v2', 'AM312_Presence_v3'],
    model: 'AM312_Presence_v3',
    vendor: 'DIY',
    description: 'DIY ESP32-C6 battery PIR presence sensor',

    extend: [
        occupancy(),
        battery(),   // percentage
        // Battery voltage = genAnalogInput.presentValue (volts) on endpoint 11.
        // The `reporting` block is REQUIRED: it makes z2m BIND genAnalogInput to the
        // coordinator during the interview. The firmware sends *bound* reports (APS
        // address mode "destination not present"), so without this binding the report
        // has nowhere to go and voltage stays null.
        numeric({
            name: 'voltage',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Battery voltage',
            unit: 'V',
            access: 'STATE_GET',
            reporting: {min: 0, max: 3600, change: 0.01},
            precision: 3,
        }),
        // Occupancy-hold setpoint = genAnalogOutput.presentValue (seconds) on endpoint 12.
        // access ALL -> z2m can WRITE it; the write is queued at the parent router and
        // collected on the sensor's next poll. The `reporting` block BINDS genAnalogOutput
        // the same way as voltage above, so the firmware's echo (the accepted, clamped
        // value) lands back in z2m and confirms the setting.
        numeric({
            name: 'occupancy_timeout',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            description:
                'Seconds to stay occupied after the last motion. The device clamps to ' +
                '1-3600 s and reports back the value it actually uses. It is a sleepy end ' +
                'device, so a change is queued at its parent and picked up on the next poll ' +
                '(a few seconds); it applies to an occupancy period already in progress.',
            unit: 's',
            access: 'ALL',
            valueMin: 1,
            valueMax: 3600,
            valueStep: 1,
            entityCategory: 'config',
            reporting: {min: 0, max: 3600, change: 1},
        }),
    ],
};
