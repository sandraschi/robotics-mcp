import {
  Navigate,
  Route,
  BrowserRouter as Router,
  Routes,
  useLocation,
} from "react-router-dom";
import { AppLayout } from "./components/layout/app-layout";
import FloatingChat from "./components/FloatingChat";
import AppsHub from "./pages/apps";
import Control from "./pages/control";
import Dashboard from "./pages/dashboard";
import DreamePage from "./pages/dreame";
import MapPage from "./pages/map";
import Onboarding, { ONBOARDING_DONE_KEY } from "./pages/onboarding";
import ROS2Page from "./pages/ros2";
import StatusPage from "./pages/status";
import ToolsExplorer from "./pages/tools";
import Logging from "./pages/Logging";
import VbotEcosystem from "./pages/vbot_ecosystem";
import YahboomPage from "./pages/yahboom";

function OnboardingGate({ children }: { children: React.ReactNode }) {
  const location = useLocation();
  const done = localStorage.getItem(ONBOARDING_DONE_KEY) === "true";
  if (done || location.pathname === "/onboarding") return <>{children}</>;
  return <Navigate to="/onboarding" replace />;
}

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/onboarding" element={<Onboarding />} />
        <Route
          path="*"
          element={
            <OnboardingGate>
              <AppLayout>
                <Routes>
                  <Route
                    path="/"
                    element={<Navigate to="/dashboard" replace />}
                  />
                  <Route path="/dashboard" element={<Dashboard />} />
                  <Route path="/status" element={<StatusPage />} />
                  <Route path="/yahboom" element={<YahboomPage />} />
                  <Route path="/dreame" element={<DreamePage />} />
                  <Route path="/ros2" element={<ROS2Page />} />
                  <Route path="/tools" element={<ToolsExplorer />} />
                  <Route path="/apps" element={<AppsHub />} />
                  <Route path="/vbot-ecosystem" element={<VbotEcosystem />} />
                  <Route path="/control" element={<Control />} />
                  <Route path="/map" element={<MapPage />} />
                  <Route path="/logs" element={<Logging />} />
                </Routes>
              </AppLayout>
            </OnboardingGate>
          }
        />
      </Routes>
      <FloatingChat />
    </Router>
  );
}

export default App;
