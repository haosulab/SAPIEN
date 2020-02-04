#pragma once

namespace sapien {

class Event {
private:
  bool cancelled = false;

  inline virtual bool isCancellable() const { return false; }
  inline virtual void cancel(){
    if (isCancellable()) {
      cancelled = true;
    }
  };
  inline virtual bool isCancelled() {
    return cancelled;
  }
};

} // namespace sapien
