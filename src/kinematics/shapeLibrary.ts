/**
 * Built-in target shapes, and the contract a generated one has to satisfy.
 *
 * A `ShapeSpec` is deliberately small: named points and the beams between
 * them. That is enough to describe anything this robot can become, and little
 * enough that a language model can emit one reliably -- which is the intended
 * pipeline. "Build me a chair" becomes a spec, and the spec goes to
 * `fitSkeleton`, which decides whether it can actually be built. The model
 * proposes; the solver disposes.
 *
 * Sizing rule, and it matters more than it looks: a beam must fit inside ONE
 * module's reach INCLUDING the offset where it welds on. A branch cannot start
 * exactly at its junction -- it starts at whatever free connector is nearest,
 * which sits a little way off -- so the chain actually has to span the beam
 * plus that offset. Cross one module's reach and `computeFeasibility` rounds up
 * to two, leaving a chain nearly twice as long as the beam it represents, which
 * has to coil to fit and drags the whole structure out of shape.
 *
 * Measured on the chair below: at 0.95x module reach it needed 13 modules for
 * 11 beams, the worst beam was off by 0.97, and the loops would not close at
 * all. At 0.8x it is one module per beam and the loops close to 2.6e-08.
 */
import { MODULE_CHAIN_LENGTH } from '../constants/geometry';
import type { ShapeSpec } from './skeletonFit';

/** One module per beam with room for the weld offset -- see the sizing rule above. */
const BEAM = MODULE_CHAIN_LENGTH * 0.8;

/**
 * Four legs, a closed seat frame, and a back -- the shape that motivated this
 * whole layer. Two independent cycles: the seat frame, and the back with the
 * seat edge it stands on. Those cycles are the interesting part; they are what
 * a tree-based robot description cannot express at all.
 */
export const CHAIR: ShapeSpec = {
  name: 'Chair',
  nodes: [
    // Feet
    { id: 'foot-fl', position: [0, 0, 0] },
    { id: 'foot-fr', position: [BEAM, 0, 0] },
    { id: 'foot-br', position: [BEAM, 0, BEAM] },
    { id: 'foot-bl', position: [0, 0, BEAM] },
    // Seat corners
    { id: 'seat-fl', position: [0, BEAM, 0] },
    { id: 'seat-fr', position: [BEAM, BEAM, 0] },
    { id: 'seat-br', position: [BEAM, BEAM, BEAM] },
    { id: 'seat-bl', position: [0, BEAM, BEAM] },
    // Back top
    { id: 'back-l', position: [0, BEAM * 2, BEAM] },
    { id: 'back-r', position: [BEAM, BEAM * 2, BEAM] },
  ],
  edges: [
    // Legs
    { from: 'foot-fl', to: 'seat-fl' },
    { from: 'foot-fr', to: 'seat-fr' },
    { from: 'foot-br', to: 'seat-br' },
    { from: 'foot-bl', to: 'seat-bl' },
    // Seat frame -- cycle 1
    { from: 'seat-fl', to: 'seat-fr' },
    { from: 'seat-fr', to: 'seat-br' },
    { from: 'seat-br', to: 'seat-bl' },
    { from: 'seat-bl', to: 'seat-fl' },
    // Back -- with the seat's back edge, cycle 2
    { from: 'seat-bl', to: 'back-l' },
    { from: 'seat-br', to: 'back-r' },
    { from: 'back-l', to: 'back-r' },
  ],
};

/** The smallest shape with a cycle -- a good first check that loops survive synthesis. */
export const TRIANGLE: ShapeSpec = {
  name: 'Triangle',
  nodes: [
    { id: 'a', position: [0, 0, 0] },
    { id: 'b', position: [BEAM, 0, 0] },
    { id: 'c', position: [BEAM / 2, BEAM * 0.87, 0] },
  ],
  edges: [
    { from: 'a', to: 'b' },
    { from: 'b', to: 'c' },
    { from: 'c', to: 'a' },
  ],
};

/** A branching shape with no cycles at all, to isolate junction handling from loop closure. */
export const TRIPOD: ShapeSpec = {
  name: 'Tripod',
  nodes: [
    { id: 'apex', position: [0, BEAM, 0] },
    { id: 'leg-a', position: [BEAM, 0, 0] },
    { id: 'leg-b', position: [-BEAM * 0.5, 0, BEAM * 0.87] },
    { id: 'leg-c', position: [-BEAM * 0.5, 0, -BEAM * 0.87] },
  ],
  edges: [
    { from: 'apex', to: 'leg-a' },
    { from: 'apex', to: 'leg-b' },
    { from: 'apex', to: 'leg-c' },
  ],
};

export const SHAPE_LIBRARY: ShapeSpec[] = [CHAIR, TRIANGLE, TRIPOD];
