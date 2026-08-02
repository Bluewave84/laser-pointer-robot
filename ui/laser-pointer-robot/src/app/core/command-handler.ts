import { inject, Service } from '@angular/core';
import { Api } from './api';
import { firstValueFrom } from 'rxjs';

@Service()
export class CommandHandler {
  private readonly api = inject(Api);

  startHoming() {
    return firstValueFrom(this.api.startHoming());
  }
  abortMotion() {
    return firstValueFrom(this.api.abortMotion());
  }
  gotoPosition(x: number, y: number) {
    return firstValueFrom(this.api.gotoPosition(x, y));
  }
  gotoPositionZero() {
    return firstValueFrom(this.api.gotoPositionZero());
  }
  setMicrosteps(value: number) {
    return firstValueFrom(this.api.setMicrosteps(value));
  }
  setSpeed(value: number) {
    return firstValueFrom(this.api.setSpeed(value));
  }
  setAcceleration(value: number) {
    return firstValueFrom(this.api.setAcceleration(value));
  }
  sampleStall() {
    return firstValueFrom(this.api.sampleStall());
  }
  testRange() {
    return firstValueFrom(this.api.testRange());
  }
  testPatternSquare() {
    return firstValueFrom(this.api.testPatternSquare());
  }
  testPatternDiamond() {
    return firstValueFrom(this.api.testPatternDiamond());
  }
  testPatternFigure8() {
    return firstValueFrom(this.api.testPatternFigure8());
  }
  testPatternSpiral() {
    return firstValueFrom(this.api.testPatternSpiral());
  }
  printCatalog() {
    return firstValueFrom(this.api.printCatalog());
  }
}
