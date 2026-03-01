
#include <benchmark/benchmark.h>

#include "core/lifecycle/dds_task.hpp"
#include "core/lifecycle/test/lifecycle_fixture.hpp"

class BenchTask
    : public core::lifecycle::
          DDSTask<core::lifecycle::TestTopicSubsList, core::lifecycle::TestTopicPubsList>
{
  public:
    using core::lifecycle::DDSTask<
        core::lifecycle::TestTopicSubsList,
        core::lifecycle::TestTopicPubsList>::DDSTask;

  protected:
    void Execute() override { benchmark::DoNotOptimize(this); }
};

// --- Simple Benchmark Test to be upgraded in future---
static void BM_TaskLifecycle(benchmark::State& state)
{
    BenchTask task("bench_task");

    for (auto _ : state)
    {
        // This times the Syncing of 'TopicCount' endpoints + Execute
        task.ExecuteStep();
        benchmark::ClobberMemory();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_TaskLifecycle)->Unit(benchmark::kNanosecond);
BENCHMARK_MAIN();
