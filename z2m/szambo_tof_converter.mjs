/**
 * Zigbee2MQTT External Converter for Szambo TOF Sensor (Native firmware)
 * Compatible with Z2M 2.x (ESM format)
 * Supports OTA updates via Z2M.
 * EP1: distance (genAnalogInput, mm)
 * EP2: battery_voltage (genAnalogInput, V)
 * EP3: measurement_interval (genAnalogOutput, min, read/write)
 * EP4: fill_level (genAnalogInput, %)
 * EP5: tank_height (genAnalogOutput, mm, read/write)
 */

const fzAnalogInput = {
    cluster: 'genAnalogInput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const endpoint = msg.endpoint.ID;
        const value = msg.data.presentValue;
        if (value === undefined || value === null || isNaN(value)) return;
        if (endpoint === 1) {
            return {distance: Math.round(value * 100) / 100};
        }
        if (endpoint === 2) {
            const voltage = Math.round(value * 100) / 100;
            const pct = Math.max(0, Math.min(100, Math.round((value - 3.0) / 1.2 * 100)));
            return {battery_voltage: voltage, battery_level: pct};
        }
        if (endpoint === 4) {
            return {fill_level: Math.round(value * 10) / 10};
        }
    },
};

const fzAnalogOutput = {
    cluster: 'genAnalogOutput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const endpoint = msg.endpoint.ID;
        if (msg.data.presentValue === undefined) return;
        if (endpoint === 3) {
            return {measurement_interval: Math.round(msg.data.presentValue)};
        }
        if (endpoint === 5) {
            return {tank_height: Math.round(msg.data.presentValue)};
        }
    },
};

const tzAnalogOutput = {
    key: ['measurement_interval', 'tank_height'],
    convertSet: async (entity, key, value, meta) => {
        if (key === 'measurement_interval') {
            const endpoint = meta.device.getEndpoint(3);
            await endpoint.write('genAnalogOutput', {presentValue: value});
            return {state: {measurement_interval: value}};
        }
        if (key === 'tank_height') {
            const endpoint = meta.device.getEndpoint(5);
            await endpoint.write('genAnalogOutput', {presentValue: value});
            return {state: {tank_height: value}};
        }
    },
    convertGet: async (entity, key, meta) => {
        if (key === 'measurement_interval') {
            const endpoint = meta.device.getEndpoint(3);
            await endpoint.read('genAnalogOutput', ['presentValue']);
        }
        if (key === 'tank_height') {
            const endpoint = meta.device.getEndpoint(5);
            await endpoint.read('genAnalogOutput', ['presentValue']);
        }
    },
};

export default {
    zigbeeModel: ['szambo_tof_sensor'],
    model: 'szambo_tof_sensor',
    vendor: 'esphome',
    description: 'Szambo TOF distance sensor (ESP32-C6 + VL53L0X) with OTA',
    fromZigbee: [fzAnalogInput, fzAnalogOutput],
    toZigbee: [tzAnalogOutput],
    exposes: [
        {type: 'numeric', name: 'distance', label: 'Distance', property: 'distance', access: 1, unit: 'mm', description: 'Distance to water surface'},
        {type: 'numeric', name: 'fill_level', label: 'Fill level', property: 'fill_level', access: 1, unit: '%', description: 'Tank fill level', value_min: 0, value_max: 100},
        {type: 'numeric', name: 'battery_voltage', label: 'Battery voltage', property: 'battery_voltage', access: 1, unit: 'V', description: 'Battery voltage', precision: 2},
        {type: 'numeric', name: 'battery_level', label: 'Battery level', property: 'battery_level', access: 1, unit: '%', description: 'Battery remaining (3.0V=0%, 4.2V=100%)', value_min: 0, value_max: 100},
        {type: 'numeric', name: 'measurement_interval', label: 'Measurement interval', property: 'measurement_interval', access: 7, unit: 'min', description: 'Measurement interval', value_min: 1, value_max: 1440},
        {type: 'numeric', name: 'tank_height', label: 'Tank height', property: 'tank_height', access: 7, unit: 'mm', description: 'Tank height (distance from sensor to empty tank bottom)', value_min: 100, value_max: 10000},
        {type: 'numeric', name: 'linkquality', label: 'Linkquality', property: 'linkquality', access: 1, unit: 'lqi', description: 'Link quality'},
    ],
    ota: true,
    meta: {multiEndpoint: true},
    endpoint: (device) => ({'1': 1, '2': 2, '3': 3, '4': 4, '5': 5}),
};
