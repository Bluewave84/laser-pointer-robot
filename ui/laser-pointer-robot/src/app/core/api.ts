import { Service, inject } from '@angular/core';
import { HttpClient } from '@angular/common/http';

@Service()
export class Api {
  private readonly http = inject(HttpClient);
  private readonly baseUrl = 'http://esp32-18d3e4';

  startHoming() {
    return this.http.post(`${this.baseUrl}/api/commands/homing.start`, {});
  }
  abortMotion() {
    return this.http.post(`${this.baseUrl}/api/commands/motion.abort`, {});
  }
  gotoPosition(x: number, y: number) {
    return this.http.post(`${this.baseUrl}/api/commands/motion.position`, { x, y });
  }
  gotoPositionZero() {
    return this.http.post(`${this.baseUrl}/api/commands/motion.zero`, {});
  }
  setMicrosteps(value: number) {
    return this.http.post(`${this.baseUrl}/api/commands/motion.microsteps`, { value });
  }
  setSpeed(value: number) {
    return this.http.post(`${this.baseUrl}/api/commands/motion.speed`, { value });
  }
  setAcceleration(value: number) {
    return this.http.post(`${this.baseUrl}/api/commands/motion.acceleration`, { value });
  }
  sampleStall() {
    return this.http.post(`${this.baseUrl}/api/commands/stall.sample`, {});
  }
  testRange() {
    return this.http.post(`${this.baseUrl}/api/commands/test.range`, {});
  }
  testPatternSquare() {
    return this.http.post(`${this.baseUrl}/api/commands/test.pattern.square`, {});
  }
  testPatternDiamond() {
    return this.http.post(`${this.baseUrl}/api/commands/test.pattern.diamond`, {});
  }
  testPatternFigure8() {
    return this.http.post(`${this.baseUrl}/api/commands/test.pattern.figure8`, {});
  }
  testPatternSpiral() {
    return this.http.post(`${this.baseUrl}/api/commands/test.pattern.spiral`, {});
  }
  printCatalog() {
    return this.http.post(`${this.baseUrl}/api/commands/catalog.print`, {});
  }
}
