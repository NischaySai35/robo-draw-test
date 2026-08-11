/** Small radian/degree formatting helpers shared by inspector inputs. */
export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;

export function toDeg(rad: number): number {
  return rad * RAD_TO_DEG;
}

export function toRad(deg: number): number {
  return deg * DEG_TO_RAD;
}

export function formatDeg(rad: number): string {
  return `${toDeg(rad).toFixed(1)}°`;
}
