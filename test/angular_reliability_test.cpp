#include <gtest/gtest.h>

#include <autonomous_flight/simulation/angularDepthReliability.h>

TEST(AngularReliability, DistinguishesPersistentCameraFaultAndHealthyBin) {
  AutoFlight::AngularDepthReliability model(2);
  std::vector<AutoFlight::AngularDepthFrameEvidence> frame(2);
  frame[0] = {10, 10, 10};
  frame[1] = {10, 0, 0};
  for (int count = 0; count < 40; ++count) model.observe(frame);
  EXPECT_EQ(model.state(0), AutoFlight::AngularReliabilityState::Healthy);
  EXPECT_EQ(model.state(1), AutoFlight::AngularReliabilityState::Faulty);
}

TEST(AngularReliability, NoReferenceAndNoReturnAbstains) {
  AutoFlight::AngularDepthReliability model(1);
  for (int count = 0; count < 40; ++count)
    model.observe({AutoFlight::AngularDepthFrameEvidence{0, 0, 0}});
  EXPECT_EQ(model.diagnosticFrames(0), 0u);
  EXPECT_EQ(model.state(0), AutoFlight::AngularReliabilityState::Unknown);
}

TEST(AngularReliability, OldFailureExpiresInsteadOfPermanentBlacklisting) {
  AutoFlight::AngularDepthReliability model(1);
  for (int count = 0; count < 40; ++count)
    model.observe({AutoFlight::AngularDepthFrameEvidence{10, 0, 0}});
  EXPECT_EQ(model.state(0), AutoFlight::AngularReliabilityState::Faulty);
  for (int count = 0; count < 60; ++count)
    model.observe({AutoFlight::AngularDepthFrameEvidence{0, 0, 0}});
  EXPECT_EQ(model.state(0), AutoFlight::AngularReliabilityState::Unknown);
}

TEST(AngularReliability, IndependentPixelFailureDoesNotBlacklistWholeBin) {
  AutoFlight::AngularDepthReliability model(1);
  for (int count = 0; count < 40; ++count)
    model.observe({AutoFlight::AngularDepthFrameEvidence{10, 7, 7}});
  EXPECT_EQ(model.state(0), AutoFlight::AngularReliabilityState::Healthy);
}
