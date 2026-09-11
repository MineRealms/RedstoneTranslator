//! Lightweight performance instrumentation and memory budget (M0.5 Commit 1).
//!
//! Counters are always active (two relaxed atomic adds per world clone) so the
//! data is available without a flag. Printing and budget checks are opt-in
//! through `--memory-budget-mb` or `MCHDL_PERF=1`. The budget uses the process
//! working-set size, so a design that would exhaust memory fails with an error
//! instead of aborting the process.

use std::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use std::time::Instant;

use crate::world::block::Block;
use crate::world::position::DimSize;

static VERBOSE: AtomicU8 = AtomicU8::new(0);
static WORLD_CLONES: AtomicUsize = AtomicUsize::new(0);
static WORLD_CLONE_BYTES: AtomicUsize = AtomicUsize::new(0);
static WORLD_ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
static LAYER_COPIES: AtomicUsize = AtomicUsize::new(0);
static LAYER_COPY_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_RSS_BYTES: AtomicUsize = AtomicUsize::new(0);
static BUDGET_BYTES: AtomicUsize = AtomicUsize::new(0);
static BUDGET_EXCEEDED: AtomicBool = AtomicBool::new(false);
static WORK_EXCEEDED: AtomicBool = AtomicBool::new(false);
static CANDIDATE_TRUTH_REJECTS: AtomicUsize = AtomicUsize::new(0);
static CANDIDATE_PORT_REJECTS: AtomicUsize = AtomicUsize::new(0);
static CANDIDATE_DRC_VIOLATIONS: AtomicUsize = AtomicUsize::new(0);
static CANDIDATE_DRC_REJECTS: AtomicUsize = AtomicUsize::new(0);
static CANDIDATE_DRC_PRE_ROUTE: AtomicUsize = AtomicUsize::new(0);

/// Maximum number of local placement generations before the search gives up.
/// A deterministic guard against the exponential local search; the error is
/// reported instead of running for minutes and producing nothing.
pub const LOCAL_WORK_LIMIT: usize = 2_000_000;

/// Maximum number of `World3D` clones a single local search may perform. Each
/// placement generation clones many worlds internally, so this is the finer
/// and more reliable work proxy.
pub const LOCAL_CLONE_LIMIT: usize = 10_000_000;

pub fn local_clone_limit() -> usize {
    std::env::var("MCHDL_LOCAL_CLONE_LIMIT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(LOCAL_CLONE_LIMIT)
}

pub fn set_verbose(enabled: bool) {
    VERBOSE.store(if enabled { 1 } else { 2 }, Ordering::Relaxed);
}

pub fn verbose() -> bool {
    match VERBOSE.load(Ordering::Relaxed) {
        1 => true,
        2 => false,
        _ => {
            let enabled = std::env::var_os("MCHDL_PERF").is_some();
            VERBOSE.store(if enabled { 1 } else { 2 }, Ordering::Relaxed);
            enabled
        }
    }
}

pub fn set_budget_mb(megabytes: usize) {
    BUDGET_BYTES.store(megabytes.saturating_mul(1024 * 1024), Ordering::Relaxed);
}

pub fn budget_bytes() -> usize {
    BUDGET_BYTES.load(Ordering::Relaxed)
}

pub fn world_bytes(size: DimSize) -> usize {
    size.0
        .saturating_mul(size.1)
        .saturating_mul(size.2)
        .saturating_mul(std::mem::size_of::<Block>())
}

pub fn record_world_alloc(size: DimSize) {
    WORLD_ALLOC_BYTES.fetch_add(world_bytes(size), Ordering::Relaxed);
}

pub fn record_world_clone(size: DimSize) {
    WORLD_CLONES.fetch_add(1, Ordering::Relaxed);
    WORLD_CLONE_BYTES.fetch_add(world_bytes(size), Ordering::Relaxed);
}

pub fn record_layer_copy(blocks: usize) {
    LAYER_COPIES.fetch_add(1, Ordering::Relaxed);
    LAYER_COPY_BYTES.fetch_add(
        blocks.saturating_mul(std::mem::size_of::<Block>()),
        Ordering::Relaxed,
    );
}

pub fn layer_copy_count() -> usize {
    LAYER_COPIES.load(Ordering::Relaxed)
}

pub fn layer_copy_bytes() -> usize {
    LAYER_COPY_BYTES.load(Ordering::Relaxed)
}

pub fn world_clone_count() -> usize {
    WORLD_CLONES.load(Ordering::Relaxed)
}

pub fn world_clone_bytes() -> usize {
    WORLD_CLONE_BYTES.load(Ordering::Relaxed)
}

pub fn world_alloc_bytes() -> usize {
    WORLD_ALLOC_BYTES.load(Ordering::Relaxed)
}

pub fn peak_rss_bytes() -> usize {
    PEAK_RSS_BYTES.load(Ordering::Relaxed)
}

pub fn budget_exceeded() -> bool {
    BUDGET_EXCEEDED.load(Ordering::Relaxed)
}

pub fn note_budget_exceeded() {
    BUDGET_EXCEEDED.store(true, Ordering::Relaxed);
}

pub fn work_exceeded() -> bool {
    WORK_EXCEEDED.load(Ordering::Relaxed)
}

pub fn note_work_exceeded() {
    WORK_EXCEEDED.store(true, Ordering::Relaxed);
}

pub fn note_candidate_truth_reject() {
    CANDIDATE_TRUTH_REJECTS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_candidate_port_reject() {
    CANDIDATE_PORT_REJECTS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_candidate_drc_violation() {
    CANDIDATE_DRC_VIOLATIONS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_candidate_drc_reject() {
    CANDIDATE_DRC_REJECTS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_candidate_drc_pre_route() {
    CANDIDATE_DRC_PRE_ROUTE.fetch_add(1, Ordering::Relaxed);
}

pub fn candidate_drc_violations() -> usize {
    CANDIDATE_DRC_VIOLATIONS.load(Ordering::Relaxed)
}

pub fn candidate_drc_rejects() -> usize {
    CANDIDATE_DRC_REJECTS.load(Ordering::Relaxed)
}

pub fn candidate_drc_pre_route() -> usize {
    CANDIDATE_DRC_PRE_ROUTE.load(Ordering::Relaxed)
}

pub fn candidate_truth_rejects() -> usize {
    CANDIDATE_TRUTH_REJECTS.load(Ordering::Relaxed)
}

pub fn candidate_port_rejects() -> usize {
    CANDIDATE_PORT_REJECTS.load(Ordering::Relaxed)
}

pub fn reset_for_tests() {
    VERBOSE.store(0, Ordering::Relaxed);
    WORLD_CLONES.store(0, Ordering::Relaxed);
    WORLD_CLONE_BYTES.store(0, Ordering::Relaxed);
    WORLD_ALLOC_BYTES.store(0, Ordering::Relaxed);
    LAYER_COPIES.store(0, Ordering::Relaxed);
    LAYER_COPY_BYTES.store(0, Ordering::Relaxed);
    PEAK_RSS_BYTES.store(0, Ordering::Relaxed);
    BUDGET_BYTES.store(0, Ordering::Relaxed);
    BUDGET_EXCEEDED.store(false, Ordering::Relaxed);
    WORK_EXCEEDED.store(false, Ordering::Relaxed);
    CANDIDATE_TRUTH_REJECTS.store(0, Ordering::Relaxed);
    CANDIDATE_PORT_REJECTS.store(0, Ordering::Relaxed);
    CANDIDATE_DRC_VIOLATIONS.store(0, Ordering::Relaxed);
    CANDIDATE_DRC_REJECTS.store(0, Ordering::Relaxed);
    CANDIDATE_DRC_PRE_ROUTE.store(0, Ordering::Relaxed);
}

pub fn rss_bytes() -> Option<usize> {
    platform_rss_bytes()
}

pub fn refresh_peak_rss() -> Option<usize> {
    let rss = rss_bytes()?;
    PEAK_RSS_BYTES.fetch_max(rss, Ordering::Relaxed);
    Some(rss)
}

pub fn rss_over_budget() -> bool {
    let budget = budget_bytes();
    if budget == 0 {
        return false;
    }
    rss_bytes().is_some_and(|rss| rss > budget)
}

pub fn check_budget(stage: &str) -> eyre::Result<()> {
    let budget = budget_bytes();
    if budget == 0 {
        return Ok(());
    }
    let Some(rss) = refresh_peak_rss() else {
        return Ok(());
    };
    if rss > budget {
        note_budget_exceeded();
        eyre::bail!(
            "memory budget exceeded during {stage}: RSS {} MiB > budget {} MiB; reduce the design or raise --memory-budget-mb",
            rss / (1024 * 1024),
            budget / (1024 * 1024)
        );
    }
    Ok(())
}

pub struct StageGuard {
    name: &'static str,
    started: Instant,
    clones: usize,
    clone_bytes: usize,
    alloc_bytes: usize,
    layer_copies: usize,
    layer_copy_bytes: usize,
}

impl Drop for StageGuard {
    fn drop(&mut self) {
        if !verbose() {
            return;
        }
        let rss = rss_bytes()
            .map(|bytes| format!("{} MiB", bytes >> 20))
            .unwrap_or_else(|| "n/a".to_owned());
        eprintln!(
            "[perf] {:<22} {:>9.1}ms clones=+{:<8} clone_bytes=+{:<12} layer_copies=+{:<8} layer_copy_bytes=+{:<12} alloc_bytes=+{:<12} rss={}",
            self.name,
            self.started.elapsed().as_secs_f64() * 1000.0,
            WORLD_CLONES
                .load(Ordering::Relaxed)
                .saturating_sub(self.clones),
            WORLD_CLONE_BYTES
                .load(Ordering::Relaxed)
                .saturating_sub(self.clone_bytes),
            LAYER_COPIES
                .load(Ordering::Relaxed)
                .saturating_sub(self.layer_copies),
            LAYER_COPY_BYTES
                .load(Ordering::Relaxed)
                .saturating_sub(self.layer_copy_bytes),
            WORLD_ALLOC_BYTES
                .load(Ordering::Relaxed)
                .saturating_sub(self.alloc_bytes),
            rss
        );
    }
}

pub fn stage(name: &'static str) -> StageGuard {
    StageGuard {
        name,
        started: Instant::now(),
        clones: WORLD_CLONES.load(Ordering::Relaxed),
        clone_bytes: WORLD_CLONE_BYTES.load(Ordering::Relaxed),
        alloc_bytes: WORLD_ALLOC_BYTES.load(Ordering::Relaxed),
        layer_copies: LAYER_COPIES.load(Ordering::Relaxed),
        layer_copy_bytes: LAYER_COPY_BYTES.load(Ordering::Relaxed),
    }
}

pub fn print_summary_if_enabled() {
    if !verbose() && budget_bytes() == 0 && !budget_exceeded() {
        return;
    }
    let rss = refresh_peak_rss()
        .map(|bytes| format!("{} MiB", bytes >> 20))
        .unwrap_or_else(|| "n/a".to_owned());
    eprintln!(
        "[perf] summary clones={} clone_bytes={} layer_copies={} layer_copy_bytes={} alloc_bytes={} candidate_truth_rejects={} candidate_port_rejects={} candidate_drc_violations={} candidate_drc_rejects={} candidate_drc_pre_route={} peak_rss={} budget={} exceeded={}",
        world_clone_count(),
        world_clone_bytes(),
        layer_copy_count(),
        layer_copy_bytes(),
        world_alloc_bytes(),
        candidate_truth_rejects(),
        candidate_port_rejects(),
        candidate_drc_violations(),
        candidate_drc_rejects(),
        candidate_drc_pre_route(),
        rss,
        budget_bytes() >> 20,
        budget_exceeded()
    );
}

#[cfg(windows)]
fn platform_rss_bytes() -> Option<usize> {
    #[repr(C)]
    struct ProcessMemoryCounters {
        cb: u32,
        page_fault_count: u32,
        peak_working_set_size: usize,
        working_set_size: usize,
        quota_peak_paged_pool_usage: usize,
        quota_paged_pool_usage: usize,
        quota_peak_non_paged_pool_usage: usize,
        quota_non_paged_pool_usage: usize,
        pagefile_usage: usize,
        peak_pagefile_usage: usize,
    }

    extern "system" {
        fn K32GetProcessMemoryInfo(
            process: *mut core::ffi::c_void,
            counters: *mut ProcessMemoryCounters,
            cb: u32,
        ) -> i32;
        fn GetCurrentProcess() -> *mut core::ffi::c_void;
    }

    unsafe {
        let mut counters = std::mem::MaybeUninit::<ProcessMemoryCounters>::zeroed().assume_init();
        counters.cb = std::mem::size_of::<ProcessMemoryCounters>() as u32;
        let ok = K32GetProcessMemoryInfo(GetCurrentProcess(), &mut counters, counters.cb);
        (ok != 0).then_some(counters.working_set_size)
    }
}

#[cfg(not(windows))]
fn platform_rss_bytes() -> Option<usize> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World3D;

    #[test]
    fn clone_counters_track_world_clones() {
        let before_clones = world_clone_count();
        let before_bytes = world_clone_bytes();
        let size = DimSize(2, 2, 2);
        let world = World3D::new(size);
        let _clone = world.clone();

        assert!(world_clone_count() > before_clones);
        assert!(world_clone_bytes() >= before_bytes + world_bytes(size));
    }

    #[test]
    fn budget_check_reports_the_stage() {
        if rss_bytes().is_none() {
            return;
        }
        let previous = budget_bytes();
        BUDGET_BYTES.store(1, Ordering::Relaxed);
        let error = check_budget("unit test").unwrap_err().to_string();
        assert!(
            error.contains("memory budget exceeded during unit test"),
            "{error}"
        );
        assert!(budget_exceeded());
        BUDGET_BYTES.store(previous, Ordering::Relaxed);
        BUDGET_EXCEEDED.store(false, Ordering::Relaxed);
    }

    #[test]
    fn zero_budget_never_fails() {
        let previous = budget_bytes();
        BUDGET_BYTES.store(0, Ordering::Relaxed);
        assert!(check_budget("unit test").is_ok());
        assert!(!rss_over_budget());
        BUDGET_BYTES.store(previous, Ordering::Relaxed);
    }
}
