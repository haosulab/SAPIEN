#include <gdkmm-3.0/gdkmm.h>
#include <gtkmm-3.0/gtkmm.h>
#include <iostream>

class MainWindow : public Gtk::ApplicationWindow {
public:
  MainWindow(BaseObjectType *obj, Glib::RefPtr<Gtk::Builder> const &builder)
      : Gtk::ApplicationWindow(obj), builder{builder} {}

  virtual ~MainWindow() = default;

private:
  Glib::RefPtr<Gtk::Builder> builder;
};

int main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "physx.4127.example");
  auto builder = Gtk::Builder::create();
  builder->add_from_file("../assets/visualizer.glade");
  MainWindow * window = nullptr;
  builder->get_widget_derived("MainWindow", window);
  auto r = app->run(*window);

  delete window;
  return r;
}
