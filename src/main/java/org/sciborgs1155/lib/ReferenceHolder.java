package org.sciborgs1155.lib;

/** A simple holder for a reference of type T. Removes the need to have static mutable fields. */
public class ReferenceHolder<T> {
  private T reference;

  public ReferenceHolder(T reference) {
    this.reference = reference;
  }

  public T get() {
    return reference;
  }

  public void set(T reference) {
    this.reference = reference;
  }
}
