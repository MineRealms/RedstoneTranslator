//! Reusable point-to-point routing engine extracted from `router.rs`.
//!
//! M2.0 moves the existing search machinery here unchanged so behavior stays
//! identical while `router.rs` keeps net ordering, fanout handling, topology
//! orchestration, and simulator validation.

mod congestion;
mod cost;
mod engine;
mod goal;
mod queue;
mod state;
pub(crate) mod validation;

pub use congestion::{CongestionConfig, CongestionMap};
pub use cost::RouteCostModel;
pub(crate) use engine::{
    adapter_allowed_contacts, adapter_touches_forbidden_existing_signal, added_route_blocks,
    isolated_output_repeater_initial_states, place_support_cobble_if_needed,
    redstone_network_positions, route_point_to_point_from_initial_state,
    route_point_to_point_with_strategy_and_allowed_contacts,
    route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength,
    routeable_output_taps, sorted_route_bounds,
};
pub use engine::{
    route_point_to_point, route_point_to_point_with_cost_model_and_congestion,
    route_point_to_point_with_strategy,
};
pub(crate) use state::{
    initial_signal_strength, is_route_terminal, powered_route_source, PoweredRouteSource,
    RouteSearchState, MAX_REDSTONE_STRENGTH,
};
pub(crate) use validation::{
    eager_route_failure_reason, first_invalid_active_route, route_candidate_powers_sink,
};
