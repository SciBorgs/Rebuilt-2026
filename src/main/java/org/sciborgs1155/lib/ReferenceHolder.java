package org.sciborgs1155.lib;

/** A simple holder for a reference of type T. Removes the need to have static mutable fields. */
public class ReferenceHolder<T> {
  private T reference;

  /** Creates a new reference holder with the given reference. */
  public ReferenceHolder(T reference) {
    this.reference = reference;
  }

  /** Gets the current reference. */
  public T get() {
    return reference;
  }

  /** Sets the reference to a new value. */
  public void set(T reference) {
    this.reference = reference;
  }
}
