<%@ include file="header.jsp" %>

          <section class="wrapper">
      <div class="row">
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                              ADMIN Login                      </header>
                          <div class="panel-body">
                          <font color="red" ><c:out value="${msg }"></c:out></font>
                              <form role="form" method="post" action="<%=request.getContextPath() %>/Admin">
                              <input type="hidden" name="action" value="doLogin" />
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Username <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="username" minlength="5" type="text" required />
                                          </div>
                                      </div>
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Password <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="password" minlength="5" type="password" required />
                                          </div>
                                      </div>
                                 
                                  <div class="form-group">
                                          <div class="col-lg-offset-2 col-lg-10">
                                              <button class="btn btn-primary" type="submit">Login</button>
                                              
                                          </div>
                                      </div>
                                 </form>
                                  </div>
                   
                      </section>
                  </div>
                  </div>
                  </section>
                  <%@ include file="footer.jsp" %>