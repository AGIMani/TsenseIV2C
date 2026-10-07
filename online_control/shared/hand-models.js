// Asset identities remain distinct; both A3.0 variants share the wire protocol.
export const A30_MODEL_IDS = Object.freeze(['TsenseI_v3_0', 'TsenseI_v3_0_withid']);
export const isA30Model = model => A30_MODEL_IDS.includes(model);
export const A32_MODEL_IDS = Object.freeze(['TsenseI_v3_2', 'TsenseI_v3_2_withid']);
export const isA32Model = model => A32_MODEL_IDS.includes(model);
export const isA3Model = model => isA30Model(model) || isA32Model(model);
